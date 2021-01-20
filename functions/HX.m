classdef HX	
	properties
		X (:,1)
		ScaleFactor(1,1) double = 1
	end
	
	methods(Static)
		function r = is(obj)
			r = isa(obj,'HX');
		end
		
		function out = scale_factor(val)
			persistent scale_factor;
			
			if isempty(scale_factor)
				scale_factor = 1;
			end
			
			if nargin
				scale_factor = val;
			end
			
			out = scale_factor;
		end
	end
	
	methods
		function obj = HX(x, rescaling_option, scale_factor)
			arguments
				x (1,:)
				rescaling_option (1,1) string {mustBeMember(rescaling_option,{'rescale','no_rescale','is_rescaled'})} = "rescale"
				scale_factor (1,1) double = HX.scale_factor
			end
			
			if rescaling_option == "rescale"
				obj.ScaleFactor = scale_factor;
				x = x * obj.ScaleFactor;
			
			elseif rescaling_option == "is_rescaled"
				obj.ScaleFactor = scale_factor;
			end
			
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
			r = HX(r(1:end-1).', "is_rescaled", obj2.ScaleFactor);
		end
		
		function obj = r(obj, f)
			%R rescales the cartesian coordinates
			obj = obj.normalize();
			obj.X(1:end-1) = obj.X(1:end-1)*f;
		end
		
		function x = cart(obj)
			obj = obj.normalize();
			
			x = obj.X(1:end-1);
			
			x = x / obj.ScaleFactor;
		end
		
		function obj = rescale(obj)
			obj.X = [obj.cart.' 1].';
			obj.ScaleFactor = 1;
		end
		
		function draw_point(obj, options)
			arguments
				obj
				options.Marker string = "x"
				options.Color string = "b"
				options.MarkerSize double = 10
			end
			
			coord = obj.cart;
			plot(coord(1), coord(2), options);
		end
		
		function draw_line(obj, limits)
			arguments
				obj
				limits(:,:) double = [-3000; 6000]
			end
			
			obj = obj.normalize();
			
			limits_size = size(limits);
			
			if limits_size(2) == 2
				l1 = HX([-1/limits(1) 0], 0);
				l2 = HX([-1/limits(2) 0], 0);
				
			elseif limits_size(1) == 2
				limits = limits.';
				
				l1 = HX([0 -1/limits(1)], "no_rescale");
				l2 = HX([0 -1/limits(2)], "no_rescale");
			end
			
			p1 = obj*l1;
			p2 = obj*l2;
			
			Seg(p1, p2).draw("Color", "m");
		end
	end
end

