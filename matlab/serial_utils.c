///////////////////////////////////////
/// Tyler Ard                       ///
/// Vehicle Mobility Systems Group  ///
/// tard(at)anl(dot)gov             ///
///////////////////////////////////////


void forceCharArrayN(const char *input, char *output, size_t n) {
    // Ensure output is exactly 'n' characters long
    memset(output, '\0', n);  // Fill with null characters
    strncpy(output, input, n); // Copy up to n characters
}