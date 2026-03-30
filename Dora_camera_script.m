clear all;

list = mobiledevlist; %mobile devices connected via matlab cloud

if isempty(list)
    disp('No mobile devices found.');
end

m = mobiledev(list.DeviceId{1}); % Access the first device's Id from the table
c = camera(m);
img = snapshot(c,'immediate');