function [ samples_X , samples_Y , nr_samples] = readData( file )
%READDATA Summary of this function goes here
%   Detailed explanation goes here
fileID = fopen(file,'r');
nr_samples = 0;

samples_X = [];
samples_Y = [];

while ~feof(fileID)
	% Loop to read file and add data to matrix.
	trq_temp = [0 ; 0 ; 0 ; 0 ; 0 ; 0];
	angle_temp = [0 ; 0 ; 0 ; 0 ; 0 ; 0];
	read_idx = 1;
    nr_samples = nr_samples + 1;

	fscanf(fileID , [ '{ sim_id = ' '%d' ' ;']);
	while (read_idx ~= 7)
		fscanf(fileID , [ ' jnt ' '%d' ' ; trq ' ]);
		trq_temp(read_idx) = fscanf(fileID , '%f');
		fscanf(fileID , [ '; agl ' '%f' ' ; trg_agl ']);
		angle_temp(read_idx) = fscanf(fileID , '%f');
		fscanf(fileID , [' ; ']);
		read_idx =  read_idx + 1;
    end
    fscanf(fileID , [ '}' '\n']);
	samples_X = [samples_X  angle_temp];
	samples_Y = [samples_Y  trq_temp];
end 

fclose(fileID);

end

