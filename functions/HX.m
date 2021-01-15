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
			elseif HX.is(obj2)
				r = obj1 * obj2.X;
			end
			r = r/r(end);
			r = HX(r(1:end-1).');
		end
		
		function obj = r(obj, f)
			%R rescales the cartesian coordinates
			obj = obj.normalize();
			obj.X(1:end-1) = obj.X(1:end-1)*f;
		end
		
		function c = cart(obj)
			obj = obj.normalize();
			c = obj.X(1:end-1);
		end
		
		
		function draw_point(obj, options)
			arguments
				obj
				options.Marker string = "x"
				options.Color string = "b"
				options.MarkerSize double = 10
			end
			
			obj = obj.normalize;
			plot(obj.X(1), obj.X(2), options);
		end
		
		function draw_line(obj, limits)
			arguments
				obj
				limits(:,:) double = [-3000; 6000]
			end
			
			obj = obj.normalize();
			
			limits_size = size(limits);
			
			if limits_size(2) == 2
				l1 = HX([-1/limits(1) 0]);
				l2 = HX([-1/limits(2) 0]);
				
			elseif limits_size(1) == 2
				limits = limits.';
				
				l1 = HX([0 -1/limits(1)]);
				l2 = HX([0 -1/limits(2)]);
			end
			
			Seg(obj*l1, obj*l2).draw("Color", "m");
		end
	end
end

