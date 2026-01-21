#ifndef _MEMPOOL_H_
#define _MEMPOOL_H_

// 内存块节点结构
typedef struct MemoryBlock{
    void *data;                 //内存块起始地址
    struct MemoryBlock *next;   //下一个内存块的地址
}MemoryBlock;

// 内存池结构
typedef struct MemoryPool{
    MemoryBlock *freeList;  //空闲内存块链表
    MemoryBlock *usedList;  //占用内存块链表
    int freeCount;          //空闲内存块数量
    int usedCount;          //占用内存块数量
    int blockCount;         //内存块总数量
}MemoryPool;

/// @brief 创建内存池
/// @param blockSize 每个内存块的大小
/// @param blockCount 内存块的数量
/// @return 内存池指针
MemoryPool *InitMemoryPool(int blockSize, int blockCount);

/// @brief 从内存池中申请一个内存块
/// @param pool 内存池
/// @return 内存块buffer指针
void *AllocateBlock(MemoryPool *pool);

/// @brief 释放内存块到内存池
/// @param pool 内存池
/// @param data 数据buffer指针
void FreeBlock(MemoryPool *pool, void *data);

/// @brief 销毁所有的内存块及分配过的空间
/// @param pool 内存池
void DestroyMemoryPool(MemoryPool *pool);

#endif



