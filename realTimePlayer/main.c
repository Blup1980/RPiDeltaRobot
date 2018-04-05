#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

//#define FAKE_TARGET

#define RT_PERIOD_US 40000UL
#define PWM_PERIOD_NS 20000000UL

#define MODEL_M 531034.0
#define MODEL_H 1550000.0


typedef struct linkedList {
	float ang[3];
	unsigned int duty[3];
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

unsigned int ang_to_duty(float angle){
	return (unsigned int)(MODEL_M*angle + MODEL_H);
}

void init_channel(int channelNb) {
	char pwm_path[35];
	char enable_path[35];
	char duty_path[35];
	char period_path[35];
	const unsigned long int pwm_period = PWM_PERIOD_NS;
	FILE *fd;

	sprintf(pwm_path,"/sys/class/pwm/pwmchip0/pwm%d",channelNb);
	sprintf(enable_path,"/sys/class/pwm/pwmchip0/pwm%d/enable",channelNb);
	sprintf(period_path,"/sys/class/pwm/pwmchip0/pwm%d/period",channelNb);

	if( access( pwm_path, F_OK ) == -1 ) {
		fd = fopen("/sys/class/pwm/pwmchip0/export","w");
		if(fd == NULL) {
			printf("can't open %s\n",pwm_path);
			exit(-2);
		}
		fprintf(fd,"%d",channelNb);
		fclose(fd);
	}

	fd = fopen(period_path,"w");
	if(fd == NULL) {
		printf("can't open %s\n",period_path);
		exit(-2);
	}
	fprintf(fd,"%lu",pwm_period);
	fclose(fd);

	fd = fopen(enable_path,"w");
	if(fd == NULL) {
		printf("can't open %s\n",enable_path);
		exit(-2);
	}
	fprintf(fd,"1");
	fclose(fd);
}


void *thread_func(void *data)
{
	struct timespec next;
	struct timespec now;

#ifndef FAKE_TARGET
	const char *pwm_path0 = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
	const char *pwm_path1 = "/sys/class/pwm/pwmchip0/pwm1/duty_cycle";
	const char *pwm_path2 = "/sys/class/pwm/pwmchip0/pwm2/duty_cycle";



	//setbuf(fd_pwm0, NULL);
	//setbuf(fd_pwm1, NULL);
	//setbuf(fd_pwm2, NULL);

#endif

	clock_gettime(CLOCK_REALTIME, &next);
	pos_list_current = pos_list_head;
	while (pos_list_current->next) {
		FILE *fd_pwm0;
		FILE *fd_pwm1;
		FILE *fd_pwm2;
		timespec_add_us(&next, RT_PERIOD_US);

		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
#ifndef FAKE_TARGET
		fd_pwm0 = fopen(pwm_path0,"w");
		if(fd_pwm0 == NULL) {
			printf("can't open %s\n",pwm_path0);
			return NULL;
		}
		fd_pwm1 = fopen(pwm_path1,"w");
		if(fd_pwm1 == NULL) {
			printf("can't open %s\n",pwm_path1);
			return NULL;
		}
		fd_pwm2 = fopen(pwm_path2,"w");
		if(fd_pwm2 == NULL) {
			printf("can't open %s\n",pwm_path2);
			return NULL;
		}
#endif

//      printf("nextTimeis: %lds, %luns\n", next.tv_sec, next.tv_nsec);


		printf("%f %f %f\n",pos_list_current->ang[0],
							pos_list_current->ang[1],
							pos_list_current->ang[2]);
#ifndef FAKE_TARGET
		//printf("writing duty of %d in channel 0\n", pos_list_current->duty[0]);
		fprintf(fd_pwm0,"%d",pos_list_current->duty[0]);
		//printf("writing duty of %d in channel 1\n", pos_list_current->duty[1]);
		fprintf(fd_pwm1,"%d",pos_list_current->duty[1]);
		//printf("writing duty of %d in channel 2\n", pos_list_current->duty[2]);
		fprintf(fd_pwm2,"%d",pos_list_current->duty[2]);
		fclose(fd_pwm0);
		fclose(fd_pwm1);
		fclose(fd_pwm2);
#endif

		pos_list_current = pos_list_current->next;
	}
	return NULL;
}


int main(int argc, char* argv[])
{
	struct sched_param param;
	pthread_attr_t attr;
	pthread_t thread;
	int ret = 0;
	char *cmd_str = NULL;
	size_t cmd_size = 0;
	float x, y, z;

	pos_list_head = malloc(sizeof(linkedList_t));
	pos_list_current = pos_list_head;

	while( getline(&cmd_str, &cmd_size, stdin) != -1) {
		if (sscanf(cmd_str,"rt-cmd:POS %f %f %f",&x, &y, &z) != 0) {

			pos_list_current->ang[0] = x;
			pos_list_current->ang[1] = y;
			pos_list_current->ang[2] = z;
			pos_list_current->duty[0] = ang_to_duty(x);
			pos_list_current->duty[1] = ang_to_duty(y);
			pos_list_current->duty[2] = ang_to_duty(z);
			pos_list_current->next = malloc(sizeof(linkedList_t));

			pos_list_current = pos_list_current->next;
			pos_list_current->next = NULL;
		}
	}
	if (cmd_str != NULL) {
		free(cmd_str);
	}

#ifndef FAKE_TARGET
	init_channel(0);
	init_channel(1);
	init_channel(2);
#endif

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
