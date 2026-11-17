#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t interrupted;

static void handle_signal(int signal_number)
{
    interrupted = signal_number;
}

static int publish_speed(const char *message, int stopping)
{
    pid_t child = fork();
    if (child < 0) {
        perror("fork");
        return -1;
    }
    if (child == 0) {
        if (stopping) {
            signal(SIGINT, SIG_IGN);
            signal(SIGTERM, SIG_IGN);
        }
        execlp("gz", "gz", "topic",
               "-t", "/astroquark/wheel_velocity",
               "-m", "gz.msgs.Double",
               "-p", message, (char *)NULL);
        perror("gz");
        _exit(127);
    }

    int status;
    while (waitpid(child, &status, 0) < 0) {
        if (errno != EINTR) {
            perror("waitpid");
            return -1;
        }
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}

int main(void)
{
    struct sigaction action = {0};
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) < 0 ||
        sigaction(SIGTERM, &action, NULL) < 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    int drive_failed = 0;
    if (!interrupted) {
        drive_failed = publish_speed("data: 1.0", 0) != 0;
        if (drive_failed && !interrupted)
            fputs("Gazebo drive command failed.\n", stderr);
    }


    for (int tick = 0; !drive_failed && !interrupted && tick < 50; ++tick) {
        struct timespec delay = {0, 100000000L};
        if (nanosleep(&delay, NULL) < 0 && errno != EINTR) {
            perror("nanosleep");
            drive_failed = 1;
        }
    }

    if (publish_speed("data: 0.0", 1) != 0) {
        fputs("Gazebo stop command failed.\n", stderr);
        return EXIT_FAILURE;
    }
    if (interrupted)
        return 128 + interrupted;
    return drive_failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
