#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>

class MessageBuffer {
    private:
        std::queue<std::string> q;
        std::condition_variable cv;
        std::mutex mutex;

        // precondition: q.size() > 0
        void send_next() {
            auto next = q.front();
            q.pop();
            std::cout << "sending " << next << "\n";
        }

    public:
        MessageBuffer() {}

        void push(const std::string& s) {
            std::unique_lock<std::mutex> lk(mutex);
            
            q.push(s);
            
            lk.unlock();
            cv.notify_all();
        }

        void send(const bool flush = false) {
            std::unique_lock<std::mutex> lk(mutex);
            cv.wait(lk, [this](){ return q.size() > 0; });
            
            send_next();
            if (flush) {
                while (q.size() > 0) {
                    send_next();
                }
            }
            lk.unlock();
        }
};

int main(int argc, char *argv[]) {
    MessageBuffer mb;

    return 0;
}