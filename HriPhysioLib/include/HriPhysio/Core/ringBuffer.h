/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#ifndef HRI_PHYSIO_CORE_RING_BUFFER_H
#define HRI_PHYSIO_CORE_RING_BUFFER_H

#include <iostream>
#include <memory>
#include <mutex>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Core {
        template <class T>
        class RingBuffer;
    }
}

template <class T>
class hriPhysio::Core::RingBuffer {
private:
    /* ============================================================================
    **  Member Variables.
    ** ============================================================================ */

    //-- The main container of the data members.
    std::unique_ptr<T[]> buffer;

    //-- Meta-data for controlling the container.
    std::size_t buffer_length;
    std::size_t buffer_head;
    std::size_t buffer_tail;
    std::size_t buffer_size;

    //-- Mutex for ensuring atomicity.
    std::mutex lock;

    //-- Enable/Disable buffer warnings.
    bool warnings = false;


public:
    /* ============================================================================
    **  Main Constructor.
    **
    ** @param length    Number of indices to allocate. [Optional arg]
    ** ============================================================================ */
    explicit RingBuffer(const std::size_t length = 0) :
        buffer_length(length),
        buffer_head(0),
        buffer_tail(0),
        buffer_size(0)
        buffer(std::make_unique<T[]>(length)) {}


    /* ============================================================================
    **  Main Destructor.
    ** ============================================================================ */
    ~RingBuffer() = default;


    /* ============================================================================
    **  Set Warnings to the passed in value (default is false).
    **
    ** @param value    Enable/Disable warnings.
    ** ============================================================================ */
    void setWarnings(bool value) {
        //-- Set the warnings flag.
        warnings = value;
    }


    /* ============================================================================
    **  Resize the internal buffer to the specified length.
    **    Note: Destroys any data left in the buffer.
    **
    ** @param length    Size of buffer to allocate.
    ** ============================================================================ */
    void resize(const std::size_t length) {

        //-- Lock the mutex to ensure read/write atomicity.
        std::scoped_lock guard(lock);

        //-- Update length, Update buffer.
        buffer_length = length;
        buffer = std::make_unique<T[]>(buffer_length);

        //-- Unlock the mutex and return.
        clear();
    }


    /* ============================================================================
    **  Clear the buffer and set all members to the default value of type.
    ** ============================================================================ */
    void clear() {

        //-- Lock the mutex to ensure read/write atomicity.
        std::scoped_lock guard(lock);
        std::fill(buffer.get(), buffer.get() + buffer_length, T{});
        //-- Set the meta-data to default.
        buffer_size = 0;
        buffer_head = 0;
        buffer_tail = 0;
    }


    /* ============================================================================
    **  Enqueue a single piece of data into the buffer.
    **
    ** @param item    Reference to a single data to insert into the buffer.
    **
    ** @return Success/Failure of the insertion.
    ** ============================================================================ */
    bool enqueue(const T& item) {
        std::scoped_lock guard(lock);
        //-- If buffer space is not allocated, exit.
        if (buffer_length == 0) {

            //-- Throw a warning if enabled.
            if (warnings) {
                std::cerr << "[DEBUG] No buffer allocated to insert into." << std::endl;
            }
            return false;
        }

        //-- If buffer is full, overwrite old data.
        if (full()) {
            if (warnings) {
                std::cerr << "[DEBUG] Buffer Overflow!! Overwriting old data." << std::endl;
            }
            //-- ``Delete`` old data.
            buffer_head = (buffer_head + 1) % buffer_length;
            --buffer_size;
        }

        //-- Insert item at the ``back``.
        buffer[buffer_tail] = item;
        //-- Update tail and size.
        buffer_tail = (buffer_tail + 1) % buffer_length;
        ++buffer_size;
        //-- Unlock the mutex and return.
        return true;
    }


    /* ============================================================================
    **  Enqueue multiple pieces of data into the buffer.
    **
    ** @param items     Array of data to insert into the buffer.
    ** @param length    Length of the array attempting to insert.
    **
    ** @return Success/Failure of the insertion.
    ** ============================================================================ */
    bool enqueue(const T* items, const std::size_t length) {

        //-- Lock the mutex to ensure read/write atomicity.
        std::scoped_lock guard(lock);
        //-- If length of items exceeds allocated space, exit.
        if (length > buffer_length || buffer_length == 0) {

            //-- Throw a warning if enabled.
            if (warnings) {
                std::cerr << "[DEBUG] Not enough buffer space allocated to insert into." << std::endl;
            }
            return false;
        }

        //-- If buffer is full, overwrite old data.
        if (buffer_size + length >= buffer_length) {

            //-- Throw a warning if enabled.
            if (warnings) {
                std::cerr << "[DEBUG] Buffer Overflow!! Overwriting old data." << std::endl;
            }

            //-- ``Delete`` old data.
            buffer_head  = (buffer_head + length) % buffer_length;
            buffer_size -= length;
        }

        //-- Insert the items to the ``back``.
        for (std::size_t idx = 0; idx < length; ++idx) {

            //-- Copy the item.
            buffer[buffer_tail] = items[idx];

            //-- Update tail and size.
            buffer_tail = (buffer_tail + 1) % buffer_length;
            ++buffer_size;
        }
        return true;
    }
    std::optional<T> dequeue() {
        std::scoped_lock guard(lock);
        if (buffer_length == 0) {
            if (warnings) {
                std::cerr << "[DEBUG] No buffer allocated to pop from." << std::endl;
            }
            return std::nullopt;
        }
        if (empty()) {
            if (warnings) {
                std::cerr << "[DEBUG] Buffer Empty!! Cannot pop." << std::endl;
            }
            return std::nullopt;
        }
        T item = buffer[buffer_head];
        buffer_head = (buffer_head + 1) % buffer_length;
        --buffer_size;
        return item;
    }


    /* ============================================================================
    **  Dequeue a single piece of data from the buffer.
    **
    ** @param item    Reference to a single data to pop from the buffer.
    **
    ** @return Success/Failure of the withdraw.
    ** ============================================================================ */
    bool dequeue(T* items, const std::size_t length, const std::size_t overlap = 0) {
        std::scoped_lock guard(lock);
        //-- If buffer space is not allocated, exit.
        if (length > buffer_length || buffer_length == 0 || overlap > length) {
            if (warnings) {
                std::cerr << "[DEBUG] No buffer allocated to pop from." << std::endl;
            }
            return false;
        }

        //-- If buffer is empty, can't pop.
        if (length > buffer_size) {
            if (warnings) {
                std::cerr << "[DEBUG] Buffer Empty!! Cannot pop." << std::endl;
            }
            return false;
        }

        std::size_t keep = length - overlap;
        std::size_t buffer_pos = buffer_head;
        for (std::size_t idx = 0; idx < length; ++idx) {
            items[idx] = buffer[buffer_pos];
            buffer_pos = (buffer_pos + 1) % buffer_length;
            if (idx < keep) {
                buffer_head = (buffer_head + 1) % buffer_length;
                --buffer_size;
            }
        }
        return true;
    }
    std::optional<T> front() const {
        std::scoped_lock guard(lock);
        if (buffer_length == 0) {
            if (warnings) {
                std::cerr << "[DEBUG] No buffer allocated to copy from." << std::endl;
            }
            return std::nullopt;
        }
        if (empty()) {
            if (warnings) {
                std::cerr << "[DEBUG] Buffer Empty!! Cannot copy." << std::endl;
            }
            return std::nullopt;
        }
        return buffer[buffer_head];
    }
    /* ============================================================================
    **  Dequeue multiple pieces of data from the buffer.
    **
    ** @param items     Array of data to write to from the buffer.
    ** @param length    Length of the array attempting to withdraw.
    **
    ** @return Success/Failure of the withdraw.
    ** ============================================================================ */
    bool front(T& item) {
        //-- Lock the mutex to ensure read/write atomicity.
        std::scoped_lock guard(lock);


        //-- If buffer space is not allocated, exit.
        if (length > buffer_length || buffer_length == 0) {
            //-- Throw a warning if enabled.
            if (warnings) {
                std::cerr << "[DEBUG] No buffer allocated to copy from." << std::endl;
            }
            return false;
        }

        //-- If buffer is less than buffer size , can't copy.
        if (length > buffer_size) {
            //-- Throw a warning if enabled.
            if (warnings) {
                std::cerr << "[DEBUG] Buffer Empty!! Cannot copy." << std::endl;
            }
            return false;
        }

        //-- Copy item at the ``front`` without changing head or size.
        std::size_t temp_head = buffer_head;
        for (std::size_t idx = 0; idx < length; ++idx) {
            items[idx] = buffer[temp_head];
            temp_head = (temp_head + 1) % buffer_length;
        }
        return true;
    }

    /* ============================================================================
    **  Get a pointer to the first element.
    **    Warning: Once you have the pointer does not guarantee atomicity!!
    **
    ** @return The pointer to the first element (or NULL if unallocated).
    ** ============================================================================ */
    T* data() {
        //-- get a pointer to the unique_ptr if allocated.
        return buffer_length != 0 ? buffer.get() : nullptr;
    }

    const T* data() const {
        //-- get a pointer to the unique_ptr if allocated.
        return buffer_length != 0 ? buffer.get() : nullptr;
    }


    /* ============================================================================
    **  Method to get if the buffer has data stored.
    **
    ** @return true if buffer is empty, false otherwise.
    ** ============================================================================ */
    bool empty() const {
        return buffer_size == 0;
    }


    /* ============================================================================
    **  Method to get if the buffer is full.
    **
    ** @return true if buffer is full, false otherwise.
    ** ============================================================================ */
    bool full() const {
        //-- Should never actually be larger, but
        //--   better safe than 3am debug sessions.
        return buffer_size >= buffer_length;
    }


    /* ============================================================================
    **  Method to get the current number of elements stored in the buffer.
    **
    ** @return number of elements stored.
    ** ============================================================================ */
    std::size_t size() const {
        return buffer_size;
    }


    /* ============================================================================
    **  Method to get the max number of elements the buffer can store.
    **
    ** @return number of elements that can be stored.
    ** ============================================================================ */
    std::size_t length() const {
        return buffer_length;
    }


private:
    /* ============================================================================
    **  Private method to handle allocating and changing internal buffer.
    **
    ** @param length    Size of buffer to allocate.
    ** ============================================================================ */
    void bufferInit(const std::size_t length) {
        //-- Delete managed data and initiallize new sized array.
        buffer = std::make_unique<T[]>(length);
    }
};

#endif /* HRI_PHYSIO_CORE_RING_BUFFER_H */

//--
//-- Authors Notes.
//--
//-- Putting definitions of a template class outside of the
//-- header file requires us  to explicitly state what data
//-- types are supported before  hand for the library to be
//-- properly linked  to  executables.  As a result we have
//-- chosen to have the definitions in this file.
//--
//-- See this post for more information:
//--   https://stackoverflow.com/questions/1022623/
//--
