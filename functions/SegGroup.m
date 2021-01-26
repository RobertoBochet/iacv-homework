classdef SegGroup
	%SEGGROUP It is a group of Seg
	
	methods(Static)
		function r = is(obj)
			r = isa(obj,'SegGroup');
		end
	end
	
	properties
		Segments(:,1) Seg
	end
	
	methods
		function obj = SegGroup(seg)
			for i=1:size(seg,1)
				obj.Segments(end+1) = Seg(HX(seg(i,1:2)), HX(seg(i,3:4))); 
			end			
		end
		
		function obj = plus(obj, seg)
			%PLUS Adds the Seg to the SegGroup
			arguments
				obj(1,1) SegGroup
				seg(1,1) Seg
			end
			
			obj.Segments(end+1) = seg;
		end
		
		function obj2 = mtimes(obj1, obj2)
			%MTIMES Returns a segment group where a linear application
			%was used on them segment
			arguments
				obj1
				obj2(1,1) SegGroup
			end
			
			for i=1:size(obj2.Segments,1)
				obj2.Segments(i) = obj1 * obj2.Segments(i); 
			end
		end
		
% 		function r = subsref(obj, index)
% 			%SUBSINDEX Returns the i-th segment
% 			arguments
% 				obj(1,1) SegGroup
% 				index
% 			end
% 			index
% 			
% 			r = obj.Segments(index);
% 		end
		
		function draw(obj, varargin)
			%DRAW Draws all the segments in the group
			for i=1:size(obj.Segments,1)
				obj.Segments(i).draw(varargin{:});
			end
		end
		
		function draw_to(obj, l, varargin)
			arguments
				obj(1,1) SegGroup
				l(1,1) HX
			end
			arguments (Repeating)
				varargin
			end
			
			for i=1:size(obj.Segments,1)
				obj.Segments(i).draw_to(l, varargin{:});
			end
		end
		
		function a = get_lines_matrix(obj)
			a = zeros(size(obj.Segments,1), size(obj.Segments(1).line.X, 1));
			
			for i=1:size(obj.Segments,1)
				a(i,:) = obj.Segments(i).line.X.';
			end
		end
		
		function [A, T] = get_normailzed_lines_matrix(obj)
			A = obj.get_lines_matrix;
			
			xr = mean(A(:,1));
			yr = mean(A(:,2));
			
			d = mean(sqrt((A(:,1) - xr).^2 + (A(:,2) - yr).^2));
			
			T = [
				[sqrt(2)/d 0 0];
				[0 sqrt(2)/d 0];
				[0 0 1];
				%[-sqrt(2)*xr/d -sqrt(2)*yr/d 1];
			];
		
			for i=1:size(A,1)
				A(i,:) = (T * A(i,:).').';
			end
		end
		
		function v = find_vanish_point(obj)
			[A, T] = obj.get_normailzed_lines_matrix;
			[~, ~, V] = svd(A);

			v = V(:, end);
			v = v / v(end);
			
			v = T'*v;

			v = HX(v(1:end-1).');
		end
	end
end

