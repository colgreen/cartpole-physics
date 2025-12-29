using System;

namespace CartPolePhysics;

internal static class ArrayMaths
{
    /// <summary>
    /// Computes dest[i] = FMA(a[i], scalar, addend[i]) for all i.
    /// </summary>
    /// <param name="dest">Destination span where results are stored.</param>
    /// <param name="addend">Values added pointwise to the product.</param>
    /// <param name="a">Values multiplied by <paramref name="scalar"/>.</param>
    /// <param name="scalar">Scalar multiplier.</param>
    /// <exception cref="ArgumentException">Thrown if spans have different lengths.</exception>
    public static void MultiplyAdd(
        Span<double> dest,
        ReadOnlySpan<double> addend,
        ReadOnlySpan<double> a,
        double scalar)
    {
        if(dest.Length != addend.Length || dest.Length != a.Length)
            throw new ArgumentException("dest, addend, and a must have the same length.");

        for(int i = 0; i < dest.Length; i++)
            dest[i] = Math.FusedMultiplyAdd(a[i], scalar, addend[i]);
    }

    /// <summary>
    /// Computes dest[i] = FMA(a[i], scalar, addend[i]) for all i.
    /// </summary>
    /// <param name="dest">Destination span where results are stored.</param>
    /// <param name="addend">Values added pointwise to the product.</param>
    /// <param name="a">Values multiplied by <paramref name="scalar"/>.</param>
    /// <param name="scalar">Scalar multiplier.</param>
    /// <exception cref="ArgumentException">Thrown if spans have different lengths.</exception>
    public static void MultiplyAdd(
        Span<float> dest,
        ReadOnlySpan<float> addend,
        ReadOnlySpan<float> a,
        float scalar)
    {
        if(dest.Length != addend.Length || dest.Length != a.Length)
            throw new ArgumentException("dest, addend, and a must have the same length.");

        for(int i = 0; i < dest.Length; i++)
            dest[i] = MathF.FusedMultiplyAdd(a[i], scalar, addend[i]);
    }
}
