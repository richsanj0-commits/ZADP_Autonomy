function z = cam_measure(truePose, P)
%CAM_MEASURE  Overhead-camera pose fix: true pose + Gaussian noise. Latency (P.cam.latency) is ignored.
z = truePose(:)' + [P.cam.posStd*randn(1,2), deg2rad(P.cam.headingStdDeg)*randn];
z(3) = atan2(sin(z(3)), cos(z(3)));
end
