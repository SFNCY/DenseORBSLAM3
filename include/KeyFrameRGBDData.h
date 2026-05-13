/**
 * @file KeyFrameRGBDData.h
 * @brief Header-only data structure for passing RGB-D frames to LocalMapping
 */
#ifndef KEYFRAME_RGBD_DATA_H
#define KEYFRAME_RGBD_DATA_H

#include <opencv2/core.hpp>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>

namespace ORB_SLAM3 {

/**
 * @brief RGB-D frame data structure for LocalMapping queue
 */
struct KeyFrameRGBD {
    cv::Mat imRGB;       // CV_8UC3 RGB image
    cv::Mat imDepth;     // CV_32F depth image (depth in meters)
    double timestamp;    // Frame timestamp
    unsigned long kfId;  // KeyFrame ID for loop closure matching

    KeyFrameRGBD() : timestamp(0.0), kfId(0) {}

    KeyFrameRGBD(cv::Mat _imRGB, cv::Mat _imDepth, double _timestamp, unsigned long _kfId)
        : imRGB(std::move(_imRGB)), imDepth(std::move(_imDepth)),
          timestamp(_timestamp), kfId(_kfId) {}
};

/**
 * @brief Thread-safe queue for RGB-D frames with overflow protection
 */
class KeyFrameRGBDQueue {
public:
    static constexpr size_t MAX_QUEUED_RGBD = 100;

    KeyFrameRGBDQueue() = default;

    // Disable copy
    KeyFrameRGBDQueue(const KeyFrameRGBDQueue&) = delete;
    KeyFrameRGBDQueue& operator=(const KeyFrameRGBDQueue&) = delete;

    /**
     * @brief Push RGB-D frame to queue (thread-safe)
     * @param frame RGB-D frame to enqueue
     * If queue exceeds MAX_QUEUED_RGBD, oldest frame is discarded
     */
    inline void Push(KeyFrameRGBD&& frame) {
        std::unique_lock<std::mutex> lock(mMutex);
        if (mQueue.size() >= MAX_QUEUED_RGBD) {
            mQueue.pop();
        }
        mQueue.push(std::move(frame));
        mCondVar.notify_one();
    }

    /**
     * @brief Pop frame from queue (thread-safe, non-blocking)
     * @param frame Output parameter for popped frame
     * @return true if frame was obtained, false if queue was empty
     */
    inline bool Pop(KeyFrameRGBD& frame) {
        std::unique_lock<std::mutex> lock(mMutex);
        if (mQueue.empty()) {
            return false;
        }
        frame = std::move(mQueue.front());
        mQueue.pop();
        return true;
    }

    /**
     * @brief Check if queue is empty
     * @return true if empty, false otherwise
     */
    inline bool Empty() {
        std::unique_lock<std::mutex> lock(mMutex);
        return mQueue.empty();
    }

    /**
     * @brief Clear all frames from queue
     */
    inline void Clear() {
        std::unique_lock<std::mutex> lock(mMutex);
        while (!mQueue.empty()) {
            mQueue.pop();
        }
    }

    /**
     * @brief Get current queue size
     * @return Number of frames in queue
     */
    inline size_t Size() {
        std::unique_lock<std::mutex> lock(mMutex);
        return mQueue.size();
    }

    /**
     * @brief Find frame by keyframe ID (for loop closure use case)
     * @param kfId KeyFrame ID to search for
     * @return Shared pointer to found frame, or nullptr if not found
     */
    inline std::shared_ptr<KeyFrameRGBD> FindByKfId(unsigned long kfId) {
        std::unique_lock<std::mutex> lock(mMutex);
        std::queue<KeyFrameRGBD> tempQueue;
        std::shared_ptr<KeyFrameRGBD> result = nullptr;

        while (!mQueue.empty()) {
            KeyFrameRGBD frame = std::move(mQueue.front());
            mQueue.pop();

            if (frame.kfId == kfId) {
                result = std::make_shared<KeyFrameRGBD>(std::move(frame));
            }
            tempQueue.push(std::move(frame));
        }

        // Restore queue
        mQueue = std::move(tempQueue);
        return result;
    }

private:
    std::queue<KeyFrameRGBD> mQueue;
    mutable std::mutex mMutex;
    std::condition_variable mCondVar;
};

} // namespace ORB_SLAM3

#endif // KEYFRAME_RGBD_DATA_H