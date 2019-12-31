namespace CartPolePhysics.Double
{
    internal static class ArrayMaths
    {
        /// <summary>
        /// Fused multiply-add.
        /// </summary>
        /// <param name="dest">Destination array. The results are stored in this array.</param>
        /// <param name="add">The elements in this array are pointwise added to the destination array.</param>
        /// <param name="a">An array to multiple by a scalar.</param>
        /// <param name="scalar">A scalar to multiply array a by.</param>
        public static void MultiplyAdd(
            double[] dest,
            double[] add,
            double[] a,
            double scalar)
        {
            // ENHANCEMENT: Consider vectorizing.
            // Vectorizing this may not be worth it as there are only 4 values, hence only a single vector op will be executed at most,
            // and if Vector<double>.Count is greater than four then we have to pad our arrays with zeros to match the wider vectors.
            for(int i=0; i < dest.Length; i++) {
                dest[i] = add[i] + (a[i] * scalar);
            }
        }
    }
}
