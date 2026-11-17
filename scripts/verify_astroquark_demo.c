
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum Column { TIME, COMMAND, WHEEL, EST_SPEED, SPEED, PITCH,
              EST_DISTANCE, DISTANCE, X, Y, Z, COLUMN_COUNT };

static const char *const names[COLUMN_COUNT] = {
    "time_s", "commanded_wheel_rad_s", "mean_wheel_rad_s",
    "estimated_speed_m_s", "measured_speed_m_s", "pitch_deg",
    "estimated_distance_m", "measured_distance_m", "x_m", "y_m", "z_m"
};

typedef struct {
    double value[COLUMN_COUNT];
} Sample;

static char *next_field(char **cursor)
{
    char *field = *cursor;
    if (field == NULL)
        return NULL;
    char *comma = strchr(field, ',');
    *cursor = comma ? comma + 1 : NULL;
    if (comma)
        *comma = '\0';
    while (isspace((unsigned char)*field))
        ++field;
    char *end = field + strlen(field);
    while (end > field && isspace((unsigned char)end[-1]))
        *--end = '\0';
    return field;
}

static int read_samples(FILE *file, Sample **samples, size_t *count)
{
    char line[4096];
    int positions[COLUMN_COUNT];
    for (int i = 0; i < COLUMN_COUNT; ++i)
        positions[i] = -1;

    if (!fgets(line, sizeof line, file) || !strchr(line, '\n'))
        return 0;
    char *cursor = line;
    if (strncmp(cursor, "\xef\xbb\xbf", 3) == 0)
        cursor += 3;
    int column_count = 0;
    char *field;
    while ((field = next_field(&cursor)) != NULL) {
        for (int i = 0; i < COLUMN_COUNT; ++i)
            if (strcmp(field, names[i]) == 0)
                positions[i] = column_count;
        ++column_count;
    }
    for (int i = 0; i < COLUMN_COUNT; ++i)
        if (positions[i] < 0) {
            fprintf(stderr, "Missing CSV column: %s\n", names[i]);
            return 0;
        }

    size_t capacity = 0;
    unsigned long line_number = 1;
    while (fgets(line, sizeof line, file)) {
        ++line_number;
        if (!strchr(line, '\n') && !feof(file))
            goto invalid_row;
        cursor = line;
        while (isspace((unsigned char)*cursor))
            ++cursor;
        if (*cursor == '\0')
            continue;

        Sample sample;
        for (int column = 0; column < column_count; ++column) {
            field = next_field(&cursor);
            if (!field)
                goto invalid_row;
            for (int i = 0; i < COLUMN_COUNT; ++i) {
                if (positions[i] != column)
                    continue;
                char *end;
                double value = strtod(field, &end);
                if (end == field || *end != '\0' || !isfinite(value))
                    goto invalid_row;
                sample.value[i] = value;
            }
        }
        if (*count == capacity) {
            size_t next = capacity ? capacity * 2 : 256;
            if (next < capacity || next > (size_t)-1 / sizeof **samples)
                return 0;
            Sample *buffer = realloc(*samples, next * sizeof **samples);
            if (!buffer)
                return 0;
            *samples = buffer;
            capacity = next;
        }
        (*samples)[(*count)++] = sample;
    }
    return !ferror(file) && *count >= 2;

invalid_row:
    fprintf(stderr, "Invalid data on CSV line %lu\n", line_number);
    return 0;
}

static int compare_double(const void *left, const void *right)
{
    double a = *(const double *)left;
    double b = *(const double *)right;
    return (a > b) - (a < b);
}

static double median(double *values, size_t count)
{
    qsort(values, count, sizeof *values, compare_double);
    return count % 2 ? values[count / 2]
                     : (values[count / 2 - 1] + values[count / 2]) / 2.0;
}

static int check(const char *name, int passed)
{
    printf("%-20s %s\n", name, passed ? "PASS" : "FAIL");
    return passed;
}

static int verify(const Sample *samples, size_t count)
{
    const double *first = samples[0].value;
    const double *last = samples[count - 1].value;
    const double ramp_start = 1.65, ramp_end = 4.126, goal = 5.25;
    const double settle_window = 0.8, tolerance = 1e-6;
    double max_gap = 0.0, peak_pitch = -INFINITY;
    double start_time = NAN, end_time = NAN;
    double max_stop_speed = 0.0, max_stop_wheel = 0.0;
    double settle_start = NAN;
    int ordered = 1, zero_commands = 1, reached_goal = 0;

    double *approach = malloc(count * sizeof *approach);
    double *settled = malloc(count * sizeof *settled);
    if (!approach || !settled) {
        fputs("Unable to allocate verification data.\n", stderr);
        free(approach);
        free(settled);
        return 0;
    }
    size_t approach_count = 0, settled_count = 0;

    for (size_t i = 0; i < count; ++i) {
        const double *row = samples[i].value;
        if (i > 0) {
            double gap = row[TIME] - samples[i - 1].value[TIME];
            ordered = ordered && gap > 0.0;
            max_gap = fmax(max_gap, gap);
        }
        if (row[X] < ramp_start - 0.5)
            approach[approach_count++] = row[Z];
        if (row[X] >= ramp_start - 0.5 && row[X] <= ramp_end + 0.5)
            peak_pitch = fmax(peak_pitch, row[PITCH]);
        if (isnan(start_time) && row[X] >= ramp_start)
            start_time = row[TIME];
        if (isnan(end_time) && row[X] >= ramp_end)
            end_time = row[TIME];
        if (row[X] >= goal)
            reached_goal = 1;

        if (last[TIME] - row[TIME] <= settle_window + tolerance) {
            if (settled_count == 0)
                settle_start = row[TIME];
            settled[settled_count++] = row[Z];
            max_stop_speed = fmax(max_stop_speed, fabs(row[SPEED]));
            max_stop_wheel = fmax(max_stop_wheel, fabs(row[WHEEL]));
            zero_commands = zero_commands && fabs(row[COMMAND]) <= tolerance;
        }
    }

    double rise = approach_count ? median(settled, settled_count) -
                                   median(approach, approach_count) : NAN;
    free(approach);
    free(settled);
    double coverage = last[TIME] - settle_start;
    double distance = last[DISTANCE];
    double relative_error = distance > 0.0
        ? fabs(last[EST_DISTANCE] - distance) / distance : NAN;

    int passed = 1;
    passed &= check("Sampling", ordered && first[TIME] >= 0 &&
                                max_gap <= 0.30 + tolerance);
    passed &= check("Ramp crossing", start_time < end_time && rise >= 0.22);
    passed &= check("Nose-up pitch", peak_pitch >= 4.0 && peak_pitch <= 14.0);
    passed &= check("Goal reached", reached_goal && last[X] >= goal);
    passed &= check("Settled stop", ordered &&
                    coverage >= settle_window - max_gap - tolerance &&
                    coverage >= settle_window / 2 && zero_commands &&
                    max_stop_speed <= 0.02 && max_stop_wheel <= 0.15);
    passed &= check("Distance agreement",
                    distance >= goal - first[X] - 0.05 && relative_error <= 0.12);

    printf("\nSamples: %zu | Final x: %.3f m | Final speed: %.3f m/s\n",
           count, last[X], last[SPEED]);
    printf("Measured distance: %.3f m | Estimated distance: %.3f m\n",
           distance, last[EST_DISTANCE]);
    printf("Overall: %s\n", passed ? "PASS" : "FAIL");
    return passed;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s RUN.csv\n", argv[0]);
        return 2;
    }
    FILE *file = fopen(argv[1], "r");
    if (!file) {
        perror(argv[1]);
        return 2;
    }
    Sample *samples = NULL;
    size_t count = 0;
    int loaded = read_samples(file, &samples, &count);
    fclose(file);
    if (!loaded) {
        fputs("Could not read at least two valid telemetry samples.\n", stderr);
        free(samples);
        return 2;
    }
    int passed = verify(samples, count);
    free(samples);
    return passed ? 0 : 1;
}
