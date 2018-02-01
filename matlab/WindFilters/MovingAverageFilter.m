function [ outputVector ] = MovingAverageFilter( inputVector, N )
%MovingAverageFilter 
%   Creates the moving average over the past N samples of inputVector

%Initialise ring buffer
head = N;
tail = 1;
lastAverage = inputVector(1);
buffer = ones(N,1)*lastAverage;

outputVector = ones(size(inputVector));

for i = 1:size(inputVector)
   newAverage = lastAverage + (inputVector(i)-buffer(tail+1))/N;
   head = mod((head+1),N);
   tail = mod((tail+1),N);
   buffer(head+1) = inputVector(i);
   lastAverage = newAverage;
   outputVector(i) = newAverage;
end


end

