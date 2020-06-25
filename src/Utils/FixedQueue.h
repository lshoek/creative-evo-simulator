#pragma once
#include <vector>

class FixedQueue 
{
private:
	std::vector<float> _buffer = std::vector<float>(1, 0);

	size_t _pushIndex;
	bool _bAlloc = false;

public:
	void allocate(size_t maxLen) {
		bool bValid = maxLen > 0;
		if (bValid) {
			_buffer.resize(maxLen, 0);
		}
		_bAlloc = bValid;
	}

	void push(const float& f) {
		if (_bAlloc) {
			_buffer[_pushIndex] = f;
			_pushIndex = (_pushIndex + 1) % _buffer.size();
		}
	}

	// Get copy of queue in contiguous memory block
	const std::vector<float> getBuffer() const {
		if (!_bAlloc) {
			return _buffer;
		}
		size_t curIndex = (_pushIndex - 1) % _buffer.size();
		std::vector<float> copy(_buffer.size());
		for (size_t i = 0; i < _buffer.size(); i++) {
			size_t reverseIndex = (curIndex - i) % _buffer.size();
			copy[i] = _buffer[reverseIndex];
		}
		return copy;
	}

	const size_t size() const {
		return _buffer.size();
	}

	const bool isAllocated() const {
		return _bAlloc;
	}
};
