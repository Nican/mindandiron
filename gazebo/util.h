#pragma once

#include <future>
#include <memory>

template <typename T, typename U>
struct BaseGroundProcessor 
{
	U lastProccessed;

	std::future<U> segmentFuture;

	bool Update(T input)
	{
		bool validUpdate = false;

		if(segmentFuture.valid())
		{
			auto status = segmentFuture.wait_for(std::chrono::seconds(0));

			if(status != std::future_status::ready)
				return false;

			//std::cout << "Updated out cloud!\n";
			lastProccessed = segmentFuture.get();
			segmentFuture = std::future<U>();

			validUpdate = true;
		}

		if(!segmentFuture.valid())
		{
			//std::cout << "Future does not have a valid state. Begin to process\n";

			segmentFuture = std::async(std::launch::async, [this, input](){
				auto val = this->AsyncronousUpdate(input);
				//std::cout << "Finished async proccessing\n";

				return val;
			});

		//std::cout << "Future state: " << segmentFuture->valid() << "\n";
		}

		return validUpdate;
	}

	virtual U AsyncronousUpdate(T input) = 0;
};
