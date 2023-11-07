using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

public class TypeConvert
{
    //*NOT UNION STRUCT
    public static byte[] StructToBytes(object obj) //IntPtr pointer
    {
        int size = Marshal.SizeOf(obj); // 구조체 길이
        byte[] bytes = new byte[size];
        IntPtr pointer = Marshal.UnsafeAddrOfPinnedArrayElement(bytes, 0);
        try
        {
            Marshal.StructureToPtr(obj, pointer, true); // 구조체 에서 포인터로
            Marshal.Copy(pointer, bytes, 0, size); // 포인터 에서 바이트 배열
        }
        finally
        {

        }

        return bytes;
    }

    //*NOT UNION STRUCT
    public static byte[] StructToBytes(object obj, int size) //IntPtr pointer
    {
        byte[] bytes = new byte[size];
        IntPtr pointer = Marshal.UnsafeAddrOfPinnedArrayElement(bytes, 0);
        try
        {
            Marshal.StructureToPtr(obj, pointer, true); // 구조체 에서 포인터로
            Marshal.Copy(pointer, bytes, 0, size); // 포인터 에서 바이트 배열
        }
        finally
        {

        }

        return bytes;
    }

    public static T BytesToStruct<T>(byte[] buffer) where T : struct
    {
        int size = Marshal.SizeOf(typeof(T));

        if (size > buffer.Length)
        {
            Debug.WriteLine("size = " + size + "/ length =" + buffer.Length);
            throw new Exception();
        }

        IntPtr pointer = Marshal.AllocHGlobal(size);
        Marshal.Copy(buffer, 0, pointer, size);

        T obj;
        try
        {
            obj = (T)Marshal.PtrToStructure(pointer, typeof(T));
        }
        finally
        {
            Marshal.FreeHGlobal(pointer);
        }

        return obj;
    }

    public static ushort SizeOfStruct<T>() where T : struct
    {
        return (ushort)Marshal.SizeOf(typeof(T));
    }
}