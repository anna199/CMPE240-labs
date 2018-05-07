/*
 * queue.h
 *
 *  Created on: May 1, 2018
 *      Author: Ran
 */

#ifndef QUEUE_H_
#define QUEUE_H_

typedef struct Queue* Queue;

extern int isFull(struct Queue* queue);
extern int getSize(struct Queue* queue);
extern int isEmpty(struct Queue* queue);
extern void enqueue(struct Queue* queue, int item);
extern int dequeue(struct Queue* queue);
extern int front(struct Queue* queue);

#endif /* QUEUE_H_ */
