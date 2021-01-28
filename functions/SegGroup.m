classdef SegGroup
	%SEGGROUP It is a group of Seg
	
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
		
		function draw(obj, varargin)
			%DRAW Draws all the segments in the group
			for i=1:size(obj.Segments,1)
				obj.Segments(i).draw(varargin{:});
			end
		end
		
		function draw_to(obj, l, varargin)
			%DRAW_TO Draws the segment until they intersect a line
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
		
		function T = get_normalized_transformation(obj)
			%GET_NORMALIZED_TRANSFORMATION Retrieves a transformation to normalized the segments points
			P = zeros(2 * size(obj.Segments,1), 3);
			
			for i=1:size(obj.Segments,1)
				P(2*i-1,:) = obj.Segments(i).P(1).X;
				P(2*i,:) = obj.Segments(i).P(2).X;
			end
			
			T = get_normalized_transformation(P);			
			T = inv(T)';
		end
		
		function v = find_vanish_point(obj)
			%FIND_VANISH_POINT Return the vanish point given by the prolungation of the segments
			T = obj.get_normalized_transformation;
			
			A = zeros(size(obj.Segments,1), 3);
			
			for i=1:size(obj.Segments,1)
				A(i,:) = (T * obj.Segments(i).line.X)';
			end			
			
			[~, ~, V] = svd(A);

			v = V(:, end);
			v = v / v(end);
			
			v = T'*v;

			v = HX(v(1:end-1).');
		end
	end
end

