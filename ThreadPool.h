#pragma once
#include <iostream>
#include <stack>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <chrono>

using namespace std;


template <typename T1>

class ThreadPool {

private:
	function<void(T1)> operation;

	stack<T1> tasks;
	vector<thread> workers;
	mutex mtx;
	condition_variable cv;
	bool stop = false;

public:
	ThreadPool(size_t numThreads, function<void(T1)> t) {
		operation = t;
		for (size_t i = 0; i < numThreads; ++i) {
			workers.emplace_back(&worker, this);
		}
	}


	~ThreadPool() {
		{
			unique_lock<mutex> lock(mtx);
			stop = true;
		}
		cv.notify_all();
		for (thread &worker : workers) {
			worker.join();
		}
	}


	void addTask(T1 args) {
		{
			unique_lock<mutex> lock(mtx);
			tasks.push(args);
		}
		cv.notify_one();
	}


	void worker() {
		while (true) {
			T1 task;
			{
				unique_lock<mutex> lock(mtx);
				cv.wait(lock, [this] { return stop || !tasks.empty(); });
				if (stop && tasks.empty()) return;
				task = tasks.top();
				tasks.pop();
			}
			operation(tasks);
		}
	}
};