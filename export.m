function export( name, frames )
%EXPORT Summary of this function goes here
%   Detailed explanation goes here

vidObj = VideoWriter(name, 'MPEG-4');
vidObj.Quality = 100;
vidObj.FrameRate = 30;
open(vidObj);
writeVideo(vidObj, frames);
close(vidObj); 

end

