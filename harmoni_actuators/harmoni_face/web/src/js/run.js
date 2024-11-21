/*startFace(
    '#D7E4F5', //background color
    '', //ros_master_uri
    .009, //cm per pixel
    0, //viseme compression factor
    '#ffffff', '#18970B', 150, 80, 400, 80, .7, "round", //eyes (white_color, iris_color, size, height, separation, iris_size, pupil_scale, pupil_shape)
    150, 130, .4, //eyelids (width, height, arch)
    '#ff99cc', 0, 10, 0, 0, //nose (color, x, y, width, height)
    '#2c241b', 15, -60, 350, 20, 18, 2, 8, 100.2, 1000.2, //mouth (color, x, y, width, height, thickness, opening, dimple_size, ulip_h_scale, llip_h_scale)
    '#2c241b', 110, 120, 18, 11 //brows (color, width, height, thickness, innersize)
)
*/
    
startFace(
    '#000000', //background color
    '', //ros_master_uri
    .009, //cm per pixel
    0, //viseme compression factor
    '#ffffff', '#85e5e5', 125, 80, 250, 70, .7, "round", //eyes (white_color, iris_color, size, height, separation, iris_size, pupil_scale, pupil_shape)
    125, 100, .4, //eyelids (width, height, arch)
    '#ff99cc', 0, 10, 0, 0, //nose (color, x, y, width, height)
    '#DCDCDC', 8, -80, 100, 10, 9, 1, 4, 50.1, 500.1, //mouth (color, x, y, width, height, thickness, opening, dimple_size, ulip_h_scale, llip_h_scale)
    '#DCDCDC', 55, 60, 9, 5, //brows (color, width, height, thickness, innersize)
    cloud_x=300, //Thinking cloud parameters
    cloud_y=60,
    cloud_size=100,
    cloud_spacing=5,
    small_cloud_x_pos=0.5,
    small_cloud_y_pos=0.01,
    small_cloud_dim=0.2,
    mid_cloud_x_pos=0.75,
    mid_cloud_y_pos=0.3,
    mid_cloud_dim=0.3,
    large_cloud_x_pos=1.5,
    large_cloud_y_pos=1,
    large_cloud_dim=1,
    appearDuration=500,
    disappearDuration=500
)