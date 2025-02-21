data = readtable('GPS20250220204100.csv')
data(1,:) = []
lat = data.Lat;
long = data.Long;

figure;
geoplot(lat, long, '-o', 'LineWidth', 2, 'MarkerSize', 5)
geobasemap('streets')
title('GPS Route on Map')

