classdef Seg
	%SEG Defines a line segment object in a projective space
	properties
		P(2,1)
	end
	
	methods
		function obj = Seg(p1,p2)
			%SEG Construct an instance of Seg
			arguments
				p1(1,1) HX
				p2(1,1) HX
			end
			
			obj.P = [p1; p2];
		end
		
		function line = line(obj)
			%LINE Returns the line to which it belongs
			% given two points, the line that passes on them is given by 
			% the cross product of the two points
			line = obj.P(1) * obj.P(2);
		end
		
		function l = length(obj)
			%LENGHT Returns the euclidian distance between the two points
			l = norm(obj.P(1).cart - obj.P(2).cart);
 		end
		
		function draw(obj, options)
			%DRAW Draws the segment
			arguments
				obj Seg
				options.Color string = "r"
				options.LineWidth = 3
			end
			
			p1 = obj.P(1).cart;
			p2 = obj.P(2).cart;
			
			plot([p1(1) p2(1)], [p1(2) p2(2)], options);
		end
		
		function draw_to(obj, line, varargin)
			%DRAW_TO Draws the segment until it intersects a line
			arguments
				obj(1,1) Seg
				line(1,1) HX
			end
			arguments (Repeating)
				varargin
			end
			
			% finds the intersection between the line and the limit
			pu = obj.line * line;
			
			s1 = Seg(obj.P(1), pu);
			s2 = Seg(obj.P(2), pu);
			
			% selects the other point that is furthest away
			if s1.length > s2.length
				s = s1;
			else
				s = s2;
			end
			
			s.draw(varargin{:});
		end
		
		function r = mtimes(obj1, obj2)
			%MTIMES Returns a segment where a linear application
			%was used on them points
			arguments
				obj1
				obj2(1,1) Seg
			end
			
			r = Seg(obj1 * obj2.P(1), obj1 * obj2.P(2));
		end
		
		function r = and(obj1, obj2)
			%AND Returns the intersection of the associated 
			%lines of two segments
			arguments
				obj1(1,1) Seg
				obj2(1,1) Seg
			end
			
			% the intersection of two lines is given by them cross product
			r = obj1.line * obj2.line;
		end
	end
end

