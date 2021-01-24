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
		
		function out = drawing_limits(val)
			persistent drawing_limits;
			
			if isempty(drawing_limits)
				drawing_limits = [[0, 3968];[0 2976]];
			end
			
			if nargin
				drawing_limits = val;
			end
			
			out = drawing_limits;
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
		
		function draw_line(obj)
			arguments
				obj
				%limits(2,2) double = [-3000; 6000]
			end
			
			l = obj.normalize().X;
			
			limits = HX.drawing_limits;
			
			xmin = limits(1,1);
			ymin = limits(2,1);
			xmax = limits(1,2);
			ymax = limits(2,2);
			
			p_limits = [
					[xmin, -(1 + l(1)*xmin)/l(2)];
					[-(1 + l(2)*ymin)/l(1), ymin];
					[xmax, -(1 + l(1)*xmax)/l(2)];
					[-(1 + l(2)*ymax)/l(1), ymax];
				];
			
			p = [];
			for i = (1:size(p_limits,1))
				if xmin <= p_limits(i,1) && p_limits(i,1) <= xmax && ymin <= p_limits(i,2) && p_limits(i,2) <= ymax
					p(end+1,:) = p_limits(i,:);
				end
			end
			
 			Seg(HX(p(1,:)), HX(p(2,:))).draw("Color", "m");
		end
	end
end

