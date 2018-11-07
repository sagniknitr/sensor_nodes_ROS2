#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
 
#include <unistd.h>
 
void * threadFunc(void * arg)
{
	// Get thread Id of calling thread
	pthread_t thId = pthread_self();
	std::cout << "Thread Id from thread function : " << thId << std::endl;
	return NULL;
}
int main()
{
 
	// Getting thread id of main function
	pthread_t mainThreadId = pthread_self();
	std::cout << "Thread Id from Main function thread : " << mainThreadId
			<< std::endl;
 
	/***** Create a New Thread ******/
 
	// Thread id
	pthread_t threadId;
 
	// Create a thread that will call function threadFunc() as thread function. Also
	// pass the pointer to thread id object. This API will set the thread id in this passed argument.
	int err = pthread_create(&threadId, NULL, &threadFunc, NULL);
 
	// Check if thread is created sucessfully
	if (err)
	{
		std::cout << "Thread creation failed : " << strerror(err);
		return err;
	}
	else
		std::cout << "Thread Created with ID : " << threadId << std::endl;
 
	//Compare Main thread Id and newly created thread id
 
	bool result = pthread_equal(mainThreadId, threadId);
 
	if (result)
		std::cout << "Both Main & new thread has same thread ids" << std::endl;
	else
		std::cout << "Both Main & new thread has different thread ids"
				<< std::endl;
 
	// Do some stuff in Main Thread
 
	std::cout << "Waiting for thread " << threadId << " to exit" << std::endl;
 
	// Wait for thread to exit, pass thread id as first argument
	err = pthread_join(threadId, NULL);
	// check if joining is sucessful
	if (err)
	{
		std::cout << "Failed to join Thread : " << strerror(err) << std::endl;
		return err;
	}
 
	std::cout << "Exiting Main" << std::endl;
 
	return 0;
}
