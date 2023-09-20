# This function takes three values (sensor measurements) and returns the two that are closer together
# Written by Adam Lahouar (09/07/23)
def vote(a, b, c):
    # Calculate absolute differences
    diff_ab = abs(a - b)
    diff_ac = abs(a - c)
    diff_bc = abs(b - c)

    # Find the minimum difference
    min_diff = min(diff_ab, diff_ac, diff_bc)

    # Map differences to value pairs
    diff_pair_mapping = {
        diff_ab: (a, b),
        diff_ac: (a, c),
        diff_bc: (b, c)
    }

    # Return the value pair corresponding to the minimum difference
    return diff_pair_mapping[min_diff]
