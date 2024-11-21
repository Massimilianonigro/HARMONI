// Make an instance of Two.js and place it on the page
// Function to create a single cloud shape with multiple protrusions.
// Leaving this values for how to form cloud hardcoded cause they work. Leaving color and stroke as customization.
function createCloud(x, y, baseSize, cloud_color='#f0f0f0', cloud_stroke='None') {
    let cloudGroup = two.makeGroup();
    
    // Main ellipses forming the cloud
    let ellipse1 = two.makeEllipse(x, y, baseSize, baseSize * 0.6);
    let ellipse2 = two.makeEllipse(x - baseSize * 0.4, y + baseSize * 0.2, baseSize * 0.7, baseSize * 0.5);
    let ellipse3 = two.makeEllipse(x + baseSize * 0.4, y + baseSize * 0.2, baseSize * 0.7, baseSize * 0.5);
    
    // Style and add ellipses to the cloud group
    [ellipse1, ellipse2, ellipse3].forEach((ellipse) => {
        ellipse.fill = '#f0f0f0';
        ellipse.stroke = 'None';
        cloudGroup.add(ellipse);
    });
    
    return cloudGroup;
}

// Function to animate the appearance of small clouds in sequence
function animateThinkingCloud(smallCloud1, smallCloud2, appearDuration=500, disappearDuration=500) {
    function sequence() {
        // Step 1: No small clouds visible
        new TWEEN.Tween({ opacity: 0 })
            .to({ opacity: 0 }, disappearDuration)
            .onStart(() => { smallCloud1.opacity = 0; smallCloud2.opacity = 0; })  // Hide both small clouds
            .start();

        // Step 2: Show only smallCloud2 (middle cloud)
        new TWEEN.Tween(smallCloud2)
            .to({ opacity: 1 }, appearDuration)
            .delay(disappearDuration)
            .onStart(() => { smallCloud1.opacity = 0; smallCloud2.opacity = 1; })  // Show middle cloud only
            .start();

        // Step 3: Show both small clouds
        new TWEEN.Tween(smallCloud1)
            .to({ opacity: 1 }, appearDuration)
            .delay(disappearDuration + appearDuration)
            .onStart(() => { smallCloud1.opacity = 1; smallCloud2.opacity = 1; })  // Show both clouds
            .onComplete(sequence)  // Loop back to start
            .start();
    }
    sequence();
}

// Function to create a "thinking" cloud sequence with two smaller clouds and one larger cloud
//By manipulating the x,y and dim values for small, mid and large cloud we can obtain differently looking thinking clouds
//Defaults now work.
function addThinkingCloud(x=500,
    y=500,
    baseSize=200,
    spacing=10,
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
    disappearDuration=500) {
    let thinkingCloudGroup = two.makeGroup();
    
    // Create clouds with different sizes
    let smallCloud1 = createCloud(x + baseSize * small_cloud_x_pos + spacing, y - baseSize * small_cloud_y_pos - spacing, baseSize * small_cloud_dim);
    let smallCloud2 = createCloud(x + baseSize * mid_cloud_x_pos + spacing, y - baseSize * mid_cloud_y_pos - spacing, baseSize * mid_cloud_dim);
    let largeCloud = createCloud(x + baseSize * large_cloud_x_pos + spacing * 2, y - baseSize * large_cloud_y_pos - spacing * 2, baseSize * large_cloud_dim);
    
    // Set initial opacity for small clouds
    smallCloud1.opacity = 0;
    smallCloud2.opacity = 0;

    // Add all clouds to the thinking cloud group
    thinkingCloudGroup.add(smallCloud1, smallCloud2, largeCloud);
    
    // Apply sequence animation to clouds
    animateThinkingCloud(smallCloud1, smallCloud2, appearDuration, disappearDuration);
    
    return thinkingCloudGroup;
}


