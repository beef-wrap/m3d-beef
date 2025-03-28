using System;
using System.Interop;
using System.Diagnostics;
using System.IO;
using System.Collections;

using static m3d.m3d;

namespace example;

class Program
{
	public static int Main(String[] args)
	{
		let filedata = scope List<uint8>();
		let f = File.ReadAll("cube.m3d", filedata);

		m3d_t* myModel = m3d_load(filedata.Ptr, null, null, null);

		Debug.WriteLine($"name: {StringView(myModel.name)}");
		Debug.WriteLine($"faces: {myModel.numface}");

		m3d_free(myModel);

		return 0;
	}
}