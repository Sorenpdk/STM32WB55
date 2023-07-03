/**
  ******************************************************************************
  * @file    linkedList.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    17/06-2023 (DD/MM-YYYY)
  * @brief   Source for linkedList.h file.
   TODO: Fill this module handles.....
  *******************************************************************************
  * @copy
  MIT License

  Copyright (c) 2023 Søren Pørksen

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/* Includes -----------------------------------------------------------*/
#include "linkedList.h"

/* Defines ------------------------------------------------------------*/

/* Private function prototypes ----------------------------------------*/
static void updateNodeInNodePool(node_t* newNode);

/* Global variables ---------------------------------------------------*/
static node_t linkedList_node_pool[LINKED_LIST_SIZE];
static uint16_t u16availableNodes;

static node_t* tailPtr = NULL;
static node_t* headPtr = NULL;
/* Public functions ----------------------------------------------------*/


void linkedList_init(void)
{
  u16availableNodes = LINKED_LIST_SIZE;

  for (uint16_t u16idx = 0; u16idx < LINKED_LIST_SIZE; u16idx++)
  {
    linkedList_node_pool[u16idx].prevNodePtr = NULL;
    linkedList_node_pool[u16idx].nextNodePtr = NULL;
    linkedList_node_pool[u16idx].u8isNodeActive = 0;
    linkedList_node_pool[u16idx].u16data = 0;
  }

}

node_t* linkedList_getNewNode(void)
{
  if (u16availableNodes == 0) { return NULL; } /* break early */

  for (uint16_t u16idx = 0; u16idx < LINKED_LIST_SIZE; u16idx++)
  {
    if (!linkedList_node_pool[u16idx].u8isNodeActive)
    {
      linkedList_node_pool[u16idx].u8isNodeActive = 1;
      u16availableNodes--;

      return &linkedList_node_pool[u16idx];
    }
  }
  return NULL;
}

void linkedList_freeNode(node_t* node)
{
  node->u8isNodeActive = 0;
  u16availableNodes++;
}

void linkedList_insertAtHead(uint16_t u16data)
{
  node_t* newNode = linkedList_getNewNode();

  newNode->u16data = u16data;

  if (headPtr == NULL)
  {
    headPtr = newNode;
    tailPtr = newNode;
  }
  else
  {
    newNode->nextNodePtr = headPtr;
    headPtr->prevNodePtr = newNode;
    headPtr = newNode;
  }

  updateNodeInNodePool(newNode);
}


void linkedList_insertAtTail(uint16_t u16data)
{
  node_t* newNode = linkedList_getNewNode();

  if (newNode == NULL) { return; } /* break early */

  newNode->u16data = u16data;

  if (tailPtr == NULL)
  {
    headPtr = newNode;
    tailPtr = newNode;
  }
  else
  {
    newNode->prevNodePtr = tailPtr;
    tailPtr->nextNodePtr = newNode;
    tailPtr = newNode;
  }

  updateNodeInNodePool(newNode);
}

void linkedList_insertAtIndex(uint16_t u16data, uint16_t u16poolIndex)
{
  if (u16poolIndex > u16availableNodes) { return;  } // Invalid u16poolIndex

  if (u16poolIndex == 0)
  {
    linkedList_insertAtHead(u16data);
    return;
  }

  if (u16poolIndex == u16availableNodes)
  {
    linkedList_insertAtTail(u16data);
    return;
  }

  node_t* newNode = linkedList_getNewNode();

  if (newNode == NULL)
  {
    return; // No available nodes left
  }

  newNode->u16data = u16data;

  node_t* currentNode = headPtr;

  for (uint16_t u16idx = 0; u16idx < (u16poolIndex - 1); u16idx++)
  {
    currentNode = currentNode->nextNodePtr;
  }

  newNode->nextNodePtr = currentNode->nextNodePtr;
  newNode->prevNodePtr = currentNode;
  currentNode->nextNodePtr->prevNodePtr = newNode;
  currentNode->nextNodePtr = newNode;

  updateNodeInNodePool(newNode);
}

void linkedList_sort(void)
{
  if (headPtr == NULL || headPtr->nextNodePtr == NULL)
  {
    return; // List is empty or has only one node
  }

  int swapped;
  node_t* currentNode;
  node_t* lastNode = NULL;

  do {
    swapped = 0;
    currentNode = headPtr;

    while (currentNode->nextNodePtr != lastNode)
    {
      if (currentNode->u16data > currentNode->nextNodePtr->u16data)
      {
	uint16_t temp = currentNode->u16data;
	currentNode->u16data = currentNode->nextNodePtr->u16data;
	currentNode->nextNodePtr->u16data = temp;
	swapped = 1;
      }
      currentNode = currentNode->nextNodePtr;
    }

    lastNode = currentNode;
  } while (swapped);

  // Update the linkedList_node_pool with the sorted nodes
  node_t* tempNode = headPtr;
  uint16_t u16idx = 0;
  while (tempNode != NULL)
  {
    updateNodeInNodePool(tempNode);
    tempNode = tempNode->nextNodePtr;
    u16idx++;
  }
}


void linkedList_removeNode(node_t* newNode)
{
  if (newNode == NULL) { return; } /* break early */

  if (newNode == headPtr)
  {
    headPtr = newNode->nextNodePtr;
  }

  if (newNode == tailPtr)
  {
    tailPtr = newNode->prevNodePtr;
  }

  if (newNode->prevNodePtr != NULL)
  {
    newNode->prevNodePtr->nextNodePtr = newNode->nextNodePtr;
  }

  if (newNode->nextNodePtr != NULL)
  {
    newNode->nextNodePtr->prevNodePtr = newNode->prevNodePtr;
  }

  linkedList_freeNode(newNode);

  updateNodeInNodePool(newNode);
}


static void updateNodeInNodePool(node_t* newNode)
{
  for (uint16_t u16idx = 0; u16idx < LINKED_LIST_SIZE; u16idx++)
  {
    if (&linkedList_node_pool[u16idx] == newNode)
    {
      linkedList_node_pool[u16idx] = *newNode;
      break;
    }
  }
}

/* Private functions ---------------------------------------------------*/


/******************* (C) COPYRIGHT 2023*****END OF FILE****/
