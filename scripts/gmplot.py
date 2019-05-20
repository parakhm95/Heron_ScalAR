import gmplot 
  
latitude_list = [ 39.9406706667, 39.940713, 39.940711 ] 
longitude_list = [ -75.200318, -75.2002868333, -75.2002203333 ] 
  
gmap3 = gmplot.GoogleMapPlotter(39.9407133333 -75.2001903333, 13) 
  
# scatter method of map object  
# scatter points on the google map 
gmap3.scatter( latitude_list, longitude_list, '# FF0000', 
                              size = 10, marker = False ) 
  
# Plot method Draw a line in 
# between given coordinates 
gmap3.plot(latitude_list, longitude_list,  
           'cornflowerblue', edge_width = 2.5) 
  
gmap3.draw( "lmao.html" ) 