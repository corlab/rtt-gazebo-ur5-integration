function include_paths
	% Adds all the specified paths, and thus all containing functions, scripts and other m-files, 
	% you want to be provided automatically during your MATLAB session to the MATLAB search path. 
	% For adding a directory with all sub-directories use the function 'addpath_r' defined below
	% instead of the built-in function 'addpath'.
	% 
	% This script works great with the '-r' option of the UNIX-matlab command:
	% (1) put this script into the userpath of your MATLAB instance
	% 	  (see 'userpath' command for more help)
	% (2) start MATLAB in UNIX with 'matlab -r include_all_relevant_paths'
	%
	%
	% copyright (2010, Christian Emmerich)
    % extended by Felix Reinhart
	%
    
  	upath = '.';
    upath = pwd;
    
    restoredefaultpath;

    addpath([upath filesep 'utils']);
    addpath([upath filesep 'metric']);
    addpath([upath filesep 'learner']);
    addpath([upath filesep 'examples']);

    

end


function addpath_r( pathStr )
	addpath(pathStr);		% add current current directory

	% add all directories in 'pathStr'
	files = dir(pathStr);
	for i = 3:numel(files)	% start with index 3 because files(1)='.' and files(2)='..' 
		if files(i).isdir && ~strcmp(files(i).name(1), '.') && ~strcmp(files(i).name(1), '@')
			addpath_r( [pathStr '/' files(i).name] );
		end
	end
end



