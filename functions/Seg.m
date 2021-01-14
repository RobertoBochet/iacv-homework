classdef Seg
	%SEG Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		P(2,1)
	end
	
	methods
		function obj = Seg(p1,p2)
			%SEG Construct an instance of this class
			obj.P = [p1; p2];
		end
		
		function line = line(obj)
			%line Returns the line to which it belongs
			line = obj.P(1) * obj.P(2);
		end
		
		function draw(obj)
			p1 = obj.P(1).cart();
			p2 = obj.P(2).cart();
			
			plot([p1(1) p2(1)], [p1(2) p2(2)], "r");
		end
	end
end

