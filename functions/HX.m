classdef HX	
	properties
		X (:,1)
	end
	
	methods(Static)
		function r = is(obj)
			r = isa(obj,'HX');
		end
	end
	
	methods
		function obj = HX(x)
			obj.X = [x 1].';
		end
		
		function obj = normalize(obj)
			obj.X = obj.X/obj.X(end);
		end
		
		function r = mtimes(obj1,obj2)
			if HX.is(obj1) && HX.is(obj2)
				r = cross(obj1.X, obj2.X);
				r = r/r(end);
				r = HX(r(1:end-1).');
			elseif HX.is(obj2)
				r = obj1 * obj2.X;
			end
		end
		
		function draw_segment(obj1,obj2)
			Seg(obj1, obj2).draw();
		end
		
		function or(obj1,obj2)
			% OR Shortcut for draw_segment
			obj1.draw_segment(obj2)
		end
		
		% rescales the cartesian coordinates
		function obj = r(obj, f) 
			obj = obj.normalize();
			obj.X(1:end-1) = obj.X(1:end-1)*f;
		end
		
		function c = cart(obj)
			obj = obj.normalize();
			c = obj.X(1:end-1);
		end
		
		function draw_line(obj)
			obj = obj.normalize();
			
			limits = [-3000 6000];
			limits_size = size(limits);
			
			if limits_size(2) == 2
				l1 = HX([-1/limits(1) 0]);
				l2 = HX([-1/limits(2) 0]);
				
			elseif limits_size(1) == 2
				limits = limits.';
				
				l1 = HX([0 -1/limits(1)]);
				l2 = HX([0 -1/limits(2)]);
			end
				
			p1 = obj*l1;
			p2 = obj*l2;
			
			p1 = p1.cart();
			p2 = p2.cart();
			
			plot([p1(1),p2(1)], [p1(2),p2(2)], "g");
		end
	end
end

