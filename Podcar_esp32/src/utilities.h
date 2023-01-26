int translate(float value, float leftMin, float leftMax, int rightMin, int rightMax){
    // Figure out how 'wide' each range is
    float leftSpan = leftMax - leftMin;
    float rightSpan = rightMax - rightMin;

    // Convert the left range into a 0-1 range (float)
    float valueScaled = (value - leftMin) / leftSpan;

    // Convert the 0-1 range into a value in the right range.
    return int(rightMin + (valueScaled * rightSpan));
}