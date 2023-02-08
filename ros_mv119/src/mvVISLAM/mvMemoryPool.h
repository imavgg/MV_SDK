/***************************************************************************//**
@copyright
   Copyright 2017 Qualcomm Technologies, Inc. All rights reserved.
   Confidential & Proprietary. Qualcomm Technologies, Inc. ("QTI")
*******************************************************************************/


#ifndef __MV_MEMORYPOOL_HEADERFILE__
#define __MV_MEMORYPOOL_HEADERFILE__


#include <list>
#include <mutex>
#include <stdlib.h>

class MemoryPool
{
public:
    enum {
        VGABLOCKSIZE = 307200
    };

    typedef unsigned char DATA_TYPE;

    MemoryPool( size_t nInitialBlocks = 4, size_t blockSize = VGABLOCKSIZE )
    {
        mBlockSize = blockSize;
        mNumBlocks = nInitialBlocks;
        for( size_t i = 0; i < nInitialBlocks; ++i )
        {
            DATA_TYPE * buf = (DATA_TYPE *)malloc( mBlockSize );
            mDataList.push_back( buf );
        }
    }
    ~MemoryPool()
    {
        std::unique_lock<std::mutex> lck( mDataListMtx );
        printf( "mempool num blocks %d\n", mNumBlocks );
        for( size_t i = 0; i < mNumBlocks; ++i )
        {
            free( mDataList.back() );
            mDataList.pop_back();
        }
    }

    void * acquireBlock()
    {
        if( !mDataList.empty() )
        {
            std::unique_lock<std::mutex> lck( mDataListMtx );
            DATA_TYPE * buf = mDataList.back();
            mDataList.pop_back();
            return buf;
        }
        else
        {
            printf("mempool empty, allocating one more block, prev NumBlocks %d \n", mNumBlocks );
            DATA_TYPE * buf = (DATA_TYPE *)malloc( mBlockSize );
            mNumBlocks++;
            return buf;
        }
    }

    void resetBlockSize( size_t blockSize )
    {
        printf( "mempool cur block size %d, new block size %d\n", mBlockSize, blockSize);
        if( mBlockSize != blockSize )
        {
            std::unique_lock<std::mutex> lck( mDataListMtx );
            for( size_t i = 0; i < mNumBlocks; ++i )
            {
                free( mDataList.back() );
                mDataList.pop_back();
            }
            mBlockSize = blockSize;
            for( size_t i = 0; i < mNumBlocks; ++i )
            {
                DATA_TYPE * buf = (DATA_TYPE *)malloc( mBlockSize );
                mDataList.push_back( buf );
            }
        }
        
    }
    void releaseBlock( void * block)
    {
        std::unique_lock<std::mutex> lck( mDataListMtx );
        mDataList.push_back( (DATA_TYPE *)block );
    }

protected:
    std::list<DATA_TYPE *> mDataList; // Array of pointers with each pointing to block of BLOCKSIZE
    std::mutex mDataListMtx;
    size_t mBlockSize;
    size_t mNumBlocks;
};


#endif    // __MV_MEMORYPOOL_HEADERFILE__
