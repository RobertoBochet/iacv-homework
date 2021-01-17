classdef HX	
	properties
		X (:,1)
		IsRescaled logical
	end
	
	methods(Static)
		function r = is(obj)
			r = isa(obj,'HX');
		end
		
		function out = rescaling(val)
			persistent Rescaling;
			
			if isempty(Rescaling)
				Rescaling = 1;
			end
			
			if nargin
				Rescaling = val;
			end
			
			out = Rescaling;
		end
	end
	
	methods
		function obj = HX(x, rescaling_option)
			arguments
				x (1,:)
				rescaling_option (1,1) string {mustBeMember(rescaling_option,{'rescale','no_rescale','is_rescaled'})} = "rescale"
			end
			
			if rescaling_option == "rescale"
				x = x * HX.rescaling;
				obj.IsRescaled = 1;
			
			elseif rescaling_option == "is_rescaled"
				obj.IsRescaled = 1;
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
			
			if obj2.IsRescaled
				m = "is_rescaled";
			else
				m = "no_rescale";
			end
			
			r = r/r(end);
			r = HX(r(1:end-1).', m);
		end
		
		function obj = r(obj, f)
			%R rescales the cartesian coordinates
			obj = obj.normalize();
			obj.X(1:end-1) = obj.X(1:end-1)*f;
		end
		
		function x = cart(obj)
			obj = obj.normalize();
			
			x = obj.X(1:end-1);
			
			if obj.IsRescaled
				x = x / HX.rescaling;
			end
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
			
			p1.IsRescaled = obj.IsRescaled;
			p2.IsRescaled = obj.IsRescaled;
			
			Seg(p1, p2).draw("Color", "m");
		end
	end
end

