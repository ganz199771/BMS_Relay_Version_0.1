
#include "mempool.h"
#include <stdlib.h>

MemoryPool *InitMemoryPool(int blockSize, int blockCount)
{
    MemoryPool *pool = NULL;

    pool = (MemoryPool *)malloc(sizeof(MemoryPool));//为内存池分配空间
    if(!pool)
        return NULL;
    
    pool->freeList = NULL;
    pool->usedList = NULL;
    for(int i = 0; i < blockCount; i++)
    {
        //创建内存块节点，插入到空闲链表
        MemoryBlock * block = (MemoryBlock *)malloc(sizeof(MemoryBlock));
        block->data = malloc(blockSize);
        block->next = pool->freeList;
        pool->freeList = block;
    }
    //初始化状态
    pool->freeCount = blockCount;
    pool->usedCount = 0;
    pool->usedList = 0;
    pool->blockCount = blockCount;

    return pool;
}


void *AllocateBlock(MemoryPool *pool)
{
    if(pool->freeList == NULL || pool->freeCount == 0)
        return NULL;
    
    __asm volatile ("cpsid i" : : : "memory");

    MemoryBlock *node = pool->freeList;
    //该内存块从空闲链表删除
    pool->freeList = node->next;
    //该内存块插入到占用链表
    node->next = pool->usedList;
    pool->usedList = node;
    //更新空闲，占用状态
    pool->usedCount++;
    pool->freeCount--;

    __asm volatile ("cpsie i" : : : "memory");
    return node->data;
}


void FreeBlock(MemoryPool *pool, void *data)
{   
    __asm volatile ("cpsid i" : : : "memory");
    MemoryBlock *cur = pool->usedList;
    MemoryBlock *pre = NULL;

    //寻找给内存块的节点
    while(pre != NULL && cur->data != data)
    {
        pre = cur;
        cur = cur->next;
    }
    if(cur == NULL)
    {
        __asm volatile ("cpsie i" : : : "memory");
        return;
    }
    //将该内存块从占用链表删除
    if(pre != NULL)
        pre->next = cur->next;
    else
        pool->usedList = cur->next;
    //将该内存块插入到空闲链表
    cur->next = pool->freeList;
    pool->freeList = cur;

    pool->freeCount++;
    pool->usedCount--;

    __asm volatile ("cpsie i" : : : "memory");
    return;
}

void DestroyMemoryPool(MemoryPool *pool)
{
    MemoryBlock *pre = NULL;
    //释放所有空闲内存块空间
    while(pool->freeList != NULL)
    {
        pre = pool->freeList;
        free(pool->freeList->data);
        pool->freeList = pool->freeList->next;
        free(pre);
    }
    //释放所有占用内存块空间
    while(pool->usedList != NULL)
    {
        pre = pool->usedList;
        free(pool->usedList->data);
        pool->usedList = pool->usedList->next;
        free(pre);
    }
    //释放内存池空间
    free(pool);

    pool->freeList = NULL;
    pool->usedList = NULL;
    pool->freeCount = 0;
    pool->usedCount = 0;

    return;
}


