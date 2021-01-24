function [T, A] = get_normalized_transformation(A)

xr = mean(A(:,1));
yr = mean(A(:,2));

d = mean(sqrt((A(:,1) - xr).^2 + (A(:,2) - yr).^2));

T = [
		[sqrt(2)/d 0 0];
		[0 sqrt(2)/d 0];
		[-sqrt(2)*xr/d -sqrt(2)*yr/d 1];
	]';

for i=1:size(A,1)
	A(i,:) = (T * A(i,:).').';
end
end
