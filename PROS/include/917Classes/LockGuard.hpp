#pragma once
#include "pros/rtos.hpp"

class LockGuard {
	private:
		pros::Mutex* _pMutex;

	public:
		LockGuard(pros::Mutex* pMutex) {
			_pMutex = pMutex;
			_pMutex->take(10000000);
		}

		~LockGuard() {
			_pMutex->give();
			_pMutex = nullptr;
		}

};