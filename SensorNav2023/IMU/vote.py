# This function takes three values (sensor measurements) and returns the two that are closer together
# Written by Adam Lahouar
def vote(val1, val2, val3):
    # Calculate absolute differences
    diff_val1_val2 = abs(val1 - val2)
    diff_val1_val3 = abs(val1 - val3)
    diff_val2_val3 = abs(val2 - val3)

    # Find the minimum difference
    min_diff = min(diff_val1_val2, diff_val1_val3, diff_val2_val3)

    # Map differences to value pairs
    diff_pair_mapping = {
        diff_val1_val2: (val1, val2),
        diff_val1_val3: (val1, val3),
        diff_val2_val3: (val2, val3)
    }

    # Return the value pair corresponding to the minimum difference
    return diff_pair_mapping[min_diff]
