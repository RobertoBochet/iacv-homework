classdef Seg
	%SEG Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		P(2,1)
	end
	
	methods(Static)
		function r = is(obj)
			r = isa(obj,'Seg');
		end
	end
	
	methods
		function obj = Seg(p1,p2)
			arguments
				p1(1,1) HX
				p2(1,1) HX
			end
			%SEG Construct an instance of this class
			obj.P = [p1; p2];
		end
		
		function line = line(obj)
			%line Returns the line to which it belongs
			line = obj.P(1) * obj.P(2);
		end
		
		function l = length(obj)
			l = norm(obj.P(1).cart - obj.P(2).cart);
 		end
		
		function draw(obj, options)
			arguments
				obj
				options.Color string = "r"
			end
			
			p1 = obj.P(1).cart();
			p2 = obj.P(2).cart();
			
			plot([p1(1) p2(1)], [p1(2) p2(2)], options);
		end
		
		function r = mtimes(obj1, obj2)
			if Seg.is(obj2)				
				r = Seg(obj1 * obj2.P(1), obj1 * obj2.P(2));
			end
		end
		function r = and(obj1, obj2)
			r = obj1.line * obj2.line;
		end
	end
end

