/**
  ******************************************************************************
  * @file    linkedList.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    16/06-2023 (DD/MM-YYYY)
  * @brief   Header for linkedList.c file.
    TODO: fill this module handles....
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LINKEDLIST_H
#define __LINKEDLIST_H

/* Private define ------------------------------------------------------------*/
#define LINKED_LIST_SIZE 10

/* Includes ------------------------------------------------------------------*/
#include "common.h"
/* Exported types ------------------------------------------------------------*/
typedef struct node_t
{
    struct node_t* prevNodePtr;
    struct node_t* nextNodePtr;
    uint8_t u8isNodeActive;
    uint16_t u16data;

} node_t;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void linkedList_init(void);
void linkedList_insertAtHead(uint16_t data);
void linkedList_insertAtTail(uint16_t data);
void linkedList_insertAtIndex(uint16_t data, uint16_t index);
void linkedList_removeNode(node_t* node);
void linkedList_sort(void);
#endif /* __LINKEDLIST_H */

/******************* (C) COPYRIGHT *****END OF FILE****/
