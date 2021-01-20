classdef SegGroup
	%SEGGROUP A group of Seg
	
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
		
		function obj = plus(obj,seg)
			if SegGroup.is(obj) && Seg.is(seg)
				obj.Segments(end+1) = seg;				
			end
		end
		
		function draw(obj)
			%DRAW Draws all the segments
			for i=1:size(obj.Segments,1)
				obj.Segments(i).draw;
			end
		end
		
		function draw_to(obj, l, options)
			arguments
				obj
				l(1,1) HX
				options.Color string = "r"
			end
			
			for i=1:size(obj.Segments,1)
				obj.Segments(i).draw_to(l);
			end
		end
		
		function a = get_lines_matrix(obj)
			a = zeros(size(obj.Segments,1), size(obj.Segments(1).line.X, 1));
			
			for i=1:size(obj.Segments,1)
				a(i,:) = obj.Segments(i).line.X.';
			end
		end
		
		function v = find_vanish_point(obj)
			A = obj.get_lines_matrix;
			
			[~, ~, V] = svd(A);

			v = V(:, end);
			v = v / v(end);

			v = HX(v(1:end-1).', "is_rescaled");
		end
	end
end

