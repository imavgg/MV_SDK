/***************************************************************************//**
@internal
   Copyright (c) 2017-2019 Qualcomm Technologies, Inc.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted (subject to the limitations in the disclaimer 
   below) provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
     this list of conditions and the following disclaimer in the documentation 
     and/or other materials provided with the distribution.
   * Neither the name of Qualcomm Technologies, Inc. nor the names of its 
     contributors may be used to endorse or promote products derived from this 
     software without specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY 
   THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT 
   NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
   PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*******************************************************************************/

#ifndef __MV_MEMORYPOOL_HEADERFILE__
#define __MV_MEMORYPOOL_HEADERFILE__


#include <vector>
#include <mutex>
#include <iostream>



class MemoryPool
{
public:
    enum {
        VGABLOCKSIZE = 307200
    };

    typedef unsigned char DATA_TYPE;

    MemoryPool( size_t nInitialBlocks = 4, size_t blockSize = VGABLOCKSIZE )
    {
        for( auto i = 0; i < nInitialBlocks; ++i )
        {
            DATA_TYPE* buf = (DATA_TYPE*)malloc( blockSize );
			if( buf == nullptr )
			{
				fprintf( stderr, "MV, ERROR, out of memory allocating %zu\n", blockSize );
				exit(1);
			}
			mFreeList.push_back( buf );
            mTotalBlocksList.push_back( buf );
        }
    }

    ~MemoryPool()
    {
        auto nBlocks = mTotalBlocksList.size();
        std::cout << "MV, INFO, mempool num blocks " << nBlocks << std::endl;
        std::unique_lock<std::mutex> lck( mDataListMtx );
        for( auto i = 0; i < nBlocks; ++i )
        {
            free( mTotalBlocksList.back() );
            mTotalBlocksList.pop_back();
        }
    }

    void * acquireBlock()
    {
        std::unique_lock<std::mutex> lck( mDataListMtx );
        if( !mFreeList.empty() )
        {
            DATA_TYPE * buf = mFreeList.back();
            mFreeList.pop_back();
            return buf;
        }
        else
        {
            std::cout << "MV, INFO, mempool empty, allocating one more block, prev NumBlocks " << mTotalBlocksList.size() << std::endl;
            DATA_TYPE* buf = (DATA_TYPE*)malloc( mBlockSize );
			if( buf == nullptr )
			{
				fprintf( stderr, "MV, ERROR, out of memory allocating %zu\n", mBlockSize );
				exit(1);
			}
            mTotalBlocksList.push_back( buf );
            return buf;
        }
    }

    void releaseBlock( void * block )
    {
        std::unique_lock<std::mutex> lck( mDataListMtx );
        for( auto i = 0; i < mFreeList.size(); ++i )
            if( block == mFreeList[i] )
                return;   /// this block was already released .
        mFreeList.push_back( (DATA_TYPE *)block );
    }

    void init( size_t nBlocks, size_t blockSize )
    {
        auto numBlocks = mTotalBlocksList.size();
        if( nBlocks == numBlocks && blockSize == VGABLOCKSIZE )
            return;

        for( auto i = 0; i < numBlocks; ++i )
        {
            free( mTotalBlocksList.back() );
            mTotalBlocksList.pop_back();
        }
        mFreeList.clear();

        std::cout << "MV, INFO, mempool resizing to nBlocks"<< nBlocks <<" and block size " << blockSize << " from " << numBlocks <<" and " << VGABLOCKSIZE << std::endl;
        {
            std::unique_lock<std::mutex> lck( mDataListMtx );
            mBlockSize = blockSize;
            for( auto i = 0; i < nBlocks; ++i )
            {
                DATA_TYPE* buf = (DATA_TYPE*)malloc( mBlockSize );
				if( buf == nullptr )
				{
					fprintf( stderr, "MV, ERROR, out of memory allocating %zu\n", mBlockSize );
					exit(1);
				}
				mFreeList.push_back( buf );
                mTotalBlocksList.push_back( buf );
            }
        }       
    }

protected:
    std::vector<DATA_TYPE*> mFreeList; // Array of pointers with each pointing to block of BLOCKSIZE free list of blocks
    std::vector<DATA_TYPE*> mTotalBlocksList; // Array of pointers with each pointing to block of BLOCKSIZE
    std::mutex mDataListMtx;
};


#endif    // __MV_MEMORYPOOL_HEADERFILE__
