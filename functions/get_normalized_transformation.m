function [T, A] = get_normalized_transformation(A)
%GET_NORMALIZED_TRANSFORMATION Given a set of points returns a
%transformation to nomralize the data and the normalized data

% calculates the average values for x and y
xr = mean(A(:,1));
yr = mean(A(:,2));

% calculates the scale factor to get a variance of sqrt(2)
s = mean(std(A(:,1:2))) / sqrt(2);

% defines the similar transformation to normalize the data
T = [
		[1/s 0 0];
		[0 1/s 0];
		[-xr/s -yr/s 1];
	]';

% applies the transfomration to the given data
for i=1:size(A,1)
	A(i,:) = (T * A(i,:).').';
end

end
