/*                                                                  
 * POSIX Real Time Example
 * using a single pthread as RT thread
 */

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>

#define RT_PERIOD_NS 4000UL


typedef struct {
	float x;
	float y;
	float z;
} pos_t;

typedef struct linkedList {
	pos_t pos;
	struct linkedList *next;
} linkedList_t;


linkedList_t *pos_list_head = NULL;
linkedList_t *pos_list_current = NULL;


void timespec_add_us(struct timespec *t, long us)
{
	t->tv_nsec += us*1000;
	if (t->tv_nsec > 1000000000) {
		t->tv_nsec = t->tv_nsec - 1000000000;// + ms*1000000;
		t->tv_sec += 1;
	}
}


void *thread_func(void *data)
{
	struct timespec next;
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &next);

	pos_list_current = pos_list_head;
	while (pos_list_current) {
		timespec_add_us(&next, RT_PERIOD_NS);
//      printf("nextTimeis: %lds, %luns\n", next.tv_sec, next.tv_nsec);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);

		printf("%f %f %f\n",pos_list_current->pos.x,
						pos_list_current->pos.y,
						pos_list_current->pos.z);
		pos_list_current = pos_list_current->next;
	}
	return NULL;
}


int main(int argc, char* argv[])
{
	struct sched_param param;
	pthread_attr_t attr;
	pthread_t thread;
	int ret;
	char *cmd_str = NULL;
	size_t cmd_size = 0;
	float x, y, z;

	while( getline(&cmd_str, &cmd_size, stdin) != -1) {
		if (sscanf(cmd_str,"rt-cmd:POS %f %f %f",&x, &y, &z) != 0) {
			pos_list_current = malloc(sizeof(linkedList_t));

			pos_list_current->pos.x = x;
			pos_list_current->pos.y = y;
			pos_list_current->pos.z = z;
			pos_list_current->next = pos_list_head;
			pos_list_head = pos_list_current;
		}
	}
	if (cmd_str != NULL) {
		free(cmd_str);
	}



	/* Lock memory */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		printf("mlockall failed: %m\n");
		exit(-2);
	}

	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr);
	if (ret) {
		printf("init pthread attributes failed\n");
		exit(-2);
	}

	/* Set a specific stack size  */
	ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
	if (ret) {
		printf("pthread setstacksize failed\n");
		exit(-2);
	}

	/* Set scheduler policy and priority of pthread */
	ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (ret) {
		printf("pthread setschedpolicy failed\n");
		exit(-2);
	}
	param.sched_priority = 80;
	ret = pthread_attr_setschedparam(&attr, &param);
	if (ret) {
		printf("pthread setschedparam failed\n");
		exit(-2);
	}
	/* Use scheduling parameters of attr */
	ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
		printf("pthread setinheritsched failed\n");
		exit(-2);
	}

	/* Create a pthread with specified attributes */
	ret = pthread_create(&thread, &attr, thread_func, NULL);
	if (ret) {
		printf("create pthread failed\n");
		exit(-2);
	}

	/* Join the thread and wait until it is done */
	ret = pthread_join(thread, NULL);
	if (ret)
		printf("join pthread failed: %m\n");
		exit(-2);

	pos_list_current = pos_list_head;
	while(pos_list_current) {
		linkedList_t *to_free;
		to_free = pos_list_current;
		pos_list_current = pos_list_current->next;
		if (to_free != NULL)
			free(to_free);
	}


	return ret;
}
