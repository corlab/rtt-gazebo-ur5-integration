function r = randInRange(mn, mx, binom)
	%Generates a randm number in a specific range. If the binom flag is set,
	%random number will be generated according to a binomial distribution 
	%with its center at the mean of the range (mx - mn)/2
    if(~exist('binom','var'))
        binom=false;
    end
    if(binom)
          r = binornd(mx - mn, 0.5) + mn;
    else
          r = floor(rand*(mx - mn + 1) + mn);
    end
end
   