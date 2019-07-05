### check Silvimétrica - PIPE Google Drive for sample data at:
### /PIPE - Silvimétrica/Software/tf matrix sample data

setwd('~/Desktop/trsf0/')
require(TreeLS)

# raw point cloud - before SLAM
lasraw = readLAS('eimi_raw.laz')

# slam path
sp = read.table('eimi_slam_path.txt')
names(sp) = c('time_stamp', 'X', 'Y', 'Z', 'roll', 'pitch', 'yaw')

# corregistered point cloud - after SLAM
las = readLAS('eimi.laz')

# point cloud frames' time stamps - before slam
tm1 = lasraw@data$gpstime %>% sort %>% unique
tm1 = tm1 / 1000000

# slam path time stamps - to compare against tm1
tm2 = sp[,1] %>% sort %>% unique

# post slam time stamps - should NOT match tm1 
tm3 = las@data$gpstime %>% sort %>% unique
tm3 = tm3 / 1000000

# visual inspection to check if time stamps match among data sources
print(tm1 %>% head, digits=20)
print(tm2 %>% head, digits=20)
print(tm3 %>% head, digits=20)

# sample time reference - for quick testing
tref = 1561904934

# sample list of time stamps to check corregistration
ts = tm2[tm2 > tref & tm2 < tref+1] %T>% print(digits=20)
# tm1[tm1 > tref & tm1 < tref+1] %>% print(digits=20)
# tm3[tm3 > tref & tm3 < tref+1] %>% print(digits=20)

# point cloud to fill with transformed point cloud frames
tempCloud = data.table()
deg2rad = pi/180

# transform and merge point cloud frames to single data set
for(t in ts){
  # transformation parameters: x,y,z,roll,pitch,yaw (I'm not sure if roll,pitch,yaw are in the correct order)
  trf = sp[ sp[,1] == t, -1 ] %>% as.double
  
  # extract points at time stamp "t"
  cloudFrame = lasfilter(lasraw, gpstime/1000000 == t) %>% TreeLS:::las2xyz() %>% cbind(1)
  
  # get transformation matrix (the angles should be reordered and/or transformed somehow)
  tfmat = TreeLS:::tfMatrix( trf[4]*deg2rad, trf[5]*deg2rad, trf[6]*deg2rad, trf[1], trf[2], trf[3] )
  
  # transform point cloud frame to fit the SLAM path (rotation + translation) 
  tfcloud = t(tfmat %*% t(cloudFrame))[,-4]
  
  # merge the transformed cloud to the corregistration data set
  tempCloud %<>% rbind(tfcloud)
}

# transform data to LAS format and check visually
lastemp = tempCloud %>% TreeLS:::toLAS()
plot(lastemp)

# check corregistered data from LOAM's output for the same time interval - should be (almost) the same as the previous lastemp 
lastemp = lasfilter(las, gpstime/1000000 > tref & gpstime/1000000 < tref+1)
plot(lastemp)
