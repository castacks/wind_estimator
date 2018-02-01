function [ positionsMetric ] = LatLonToMetric( positionsLatLon, reference )

    NAUT_MILES_TO_KILOMETERS = 1/0.539957;

    lonDegToMeters = cos(degtorad(reference(1)))*NAUT_MILES_TO_KILOMETERS*60*1000; %1 degree is cos(lat)*60NM
    latDegToMeters = 60*NAUT_MILES_TO_KILOMETERS*1000; % 1 degree is 60NM
    
    positionsMetric = zeros(size(positionsLatLon,1),2);
    
    positionsMetric(:,1) = (positionsLatLon(:,1) - reference(1))*latDegToMeters;
    positionsMetric(:,2) = (positionsLatLon(:,2) - reference(2))*lonDegToMeters;
    
end

