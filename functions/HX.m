classdef HX	
	properties
		X (:,1)
	end
	
	methods(Static)
		function r = is(obj)
			r = isa(obj,'HX');
		end
		
		function out = drawing_limits(val)
			%DRAWING_LIMITS They are the limits in which can be drawn
			persistent drawing_limits;
			
			if isempty(drawing_limits)
				drawing_limits = [[0, 3968];[0 2976]];
			end
			
			if nargin
				drawing_limits = val;
			end
			
			out = drawing_limits;
		end
		
		function sdraw_point(obj)
			%SDRAW_POINT Draws the given point
			obj.draw_point();
		end
	end
	
	methods
		function obj = HX(x)
			arguments
				x (1,:)
			end
			
			obj.X = [x 1].';
		end
		
		function obj = normalize(obj)
			obj.X = obj.X/obj.X(end);
		end
		
		function r = mtimes(obj1,obj2)
			%MTIMES If provided two HX returns HX_1 x HX_2 else retruns the matrix product
			arguments
				obj1
				obj2 HX
			end
			
			if HX.is(obj1)
				r = cross(obj1.X, obj2.X);
			else
				r = obj1 * obj2.X;
			end
			
			r = r/r(end);
			r = HX(r(1:end-1).');
		end
		
		function x = cart(obj)
			%CART Returns the cartesian coordinates
			x = obj.normalize().X(1:end-1);
		end
				
		function draw_point(obj, options)
			%DRAW_POINT Draws the point
			arguments
				obj HX
				options.Marker string = "x"
				options.Color string = "b"
				options.MarkerSize double = 10
			end
			
			p = obj.cart;
			plot(p(1), p(2), options);
		end
		
		function draw_line(obj, varargin)
			%DRAW_LINE Draws the line
			
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
			
			if size(p,1) >= 2
				Seg(HX(p(1,:)), HX(p(2,:))).draw(varargin{:});
			end
		end
	end
end

