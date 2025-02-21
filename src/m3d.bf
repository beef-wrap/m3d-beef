/*
 * m3d.h
 * https://gitlab.com/bztsrc/model3d
 *
 * Copyright (C) 2020 bzt (bztsrc@gitlab)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * @brief ANSI C89 / C++11 single header importer / exporter SDK for the Model 3D (.M3D) format
 * https://gitlab.com/bztsrc/model3d
 *
 * PNG decompressor included from (with minor modifications to make it C89 valid):
 *  stb_image - v2.13 - public domain image loader - http://nothings.org/stb_image.h
 *
 * @version: 1.0.0
 */

using System;
using System.Interop;

namespace M3D_Beef;

public static class M3D
{
	#if !M3D_DOUBLE
	typealias M3D_FLOAT = float;
	#if !M3D_EPSILON
	/* carefully choosen for IEEE 754 don't change */
	const float M3D_EPSILON = ((M3D_FLOAT)1e-7);
	#endif
	#else
	typealias M3D_FLOAT = double;
	#if !M3D_EPSILON
	const double M3D_EPSILON = ((M3D_FLOAT)1e-14)
	#endif
	#endif

	#if !M3D_SMALLINDEX
	typealias M3D_INDEX = c_uint;
	typealias M3D_VOXEL = c_ushort;
	const int M3D_UNDEF = 0xffffffff;
	const int M3D_INDEXMAX = 0xfffffffe;
	const int M3D_VOXUNDEF = 0xffff;
	const int M3D_VOXCLEAR = 0xfffe;
	#else
	typealias M3D_INDEX = c_ushort;
	typealias M3D_VOXEL = c_uchar;
	const int M3D_UNDEF = 0xffff;
	const int M3D_INDEXMAX = 0xfffe;
	const int M3D_VOXUNDEF = 0xff;
	const int M3D_VOXCLEAR = 0xfe;
	#endif

	const int M3D_NOTDEFINED = 0xffffffff;

	#if !M3D_NUMBONE
	const int M3D_NUMBONE = 4;
	#endif

	#if !M3D_BONEMAXLEVEL
	const int M3D_BONEMAXLEVEL = 64;
	#endif

   /*** File format structures ***/

   /**
   * M3D file format structure
   *  3DMO m3dchunk_t file header chunk, may followed by compressed data
   *  PRVW preview chunk (optional)
   *  HEAD m3dhdr_t model header chunk
   *  n x m3dchunk_t more chunks follow
   *      CMAP color map chunk (optional)
   *      TMAP texture map chunk (optional)
   *      VRTS vertex data chunk (optional if it's a material library)
   *      BONE bind-pose skeleton, bone hierarchy chunk (optional)
   *          n x m3db_t contains propably more, but at least one bone
   *          n x m3ds_t skin group records
   *      MTRL* material chunk(s), can be more (optional)
   *          n x m3dp_t each material contains propapbly more, but at least one property
   *                     the properties are configurable with a static array, see m3d_propertytypes
   *      n x m3dchunk_t at least one, but maybe more face chunks
   *          PROC* procedural face, or
   *          MESH* triangle mesh (vertex index list) or
   *          VOXT, VOXD* voxel image (converted to mesh) or
   *          SHPE* mathematical shapes like parameterized surfaces
   *      LBLS* annotation label chunks, can be more (optional)
   *      ACTN* action chunk(s), animation-pose skeletons, can be more (optional)
   *          n x m3dfr_t each action contains probably more, but at least one frame
   *              n x m3dtr_t each frame contains probably more, but at least one transformation
   *      ASET* inlined asset chunk(s), can be more (optional)
   *  OMD3 end chunk
   *
   * Typical chunks for a game engine: 3DMO, HEAD, CMAP, TMAP, VRTS, BONE, MTRL, MESH, ACTN, OMD3
   * Typical chunks for distibution:   3DMO, PRVW, HEAD, CMAP, TMAP, VRTS, BONE, MTRL, MESH, ACTN, ASET, OMD3
   * Typical chunks for voxel image:   3DMO, HEAD, CMAP, MTRL, VOXT, VOXD, VOXD, VOXD, OMD3
   * Typical chunks for CAD software:  3DMO, PRVW, HEAD, CMAP, TMAP, VRTS, MTRL, SHPE, LBLS, OMD3
   */

	[CRepr]
	public struct m3dhdr_t
	{
		c_char[4] magic;
		c_uint length;
		float scale; /* deliberately not M3D_FLOAT */
		c_uint types;
	}

	[CRepr]
	public struct m3dchunk_t
	{
		c_char[4] magic;
		c_uint length;
	}

	 /*** in-memory model structure ***/

	 /* textmap entry */
	[CRepr]
	public struct m3dti_t
	{
		M3D_FLOAT u;
		M3D_FLOAT v;
	}

	 /* texture */
	[CRepr]
	public struct m3dtx_t
	{
		c_char* name; /* texture name */
		c_uchar* d; /* pixels data */
		c_ushort w; /* width */
		c_ushort h; /* height */
		c_uchar f; /* format, 1 = grayscale, 2 = grayscale+alpha, 3 = rgb, 4 = rgba */
	}

	[CRepr]
	public struct m3dw_t
	{
		M3D_INDEX vertexid;
		M3D_FLOAT weight;
	}

	 /* bone entry */
	[CRepr]
	public struct m3db_t
	{
		M3D_INDEX parent; /* parent bone index */
		c_char* name; /* name for this bone */
		M3D_INDEX pos; /* vertex index position */
		M3D_INDEX ori; /* vertex index orientation (quaternion) */
		M3D_INDEX numweight; /* number of controlled vertices */
		m3dw_t* weight; /* weights for those vertices */
		M3D_FLOAT[416] mat; /* transformation matrix */
	}

	 /* skin: bone per vertex entry */
	[CRepr]
	public struct m3ds_t
	{
		M3D_INDEX boneid;
		M3D_FLOAT weight;
	}

	 /* vertex entry */
	[CRepr]
	public struct m3dv_t
	{
		M3D_FLOAT x; /* 3D coordinates and weight */
		M3D_FLOAT y;
		M3D_FLOAT z;
		M3D_FLOAT w;
		c_uint color; /* default vertex color */
		M3D_INDEX skinid; /* skin index */
	 #if M3D_VERTEXTYPE
		c_uchar type;
	 #endif
	}

	 /* material property formats */
	enum maerial_property_format
	{
		m3dpf_color,
		m3dpf_uint8,
		m3dpf_uint16,
		m3dpf_uint32,
		m3dpf_float,
		m3dpf_map
	};

	[CRepr]
	public struct m3dpd_t
	{
		c_uchar format;
		c_uchar id;

	 #if M3D_ASCII
		#define M3D_PROPERTYDEF(f,i,n) { (f), (i), (char*)(n) }
		c_char* key;
	 #endif

	 /*#if !M3D_ASCII
		#define M3D_PROPERTYDEF(f,i,n) { (f), (i) }
	 #endif*/
	}

	 /* material property types */
	 /* You shouldn't change the first 8 display and first 4 physical property. Assign the rest as you like. */
	[AllowDuplicates]
	enum material_property_type
	{
		m3dp_Kd = 0, /* scalar display properties */
		m3dp_Ka,
		m3dp_Ks,
		m3dp_Ns,
		m3dp_Ke,
		m3dp_Tf,
		m3dp_Km,
		m3dp_d,
		m3dp_il,

		m3dp_Pr = 64, /* scalar physical properties */
		m3dp_Pm,
		m3dp_Ps,
		m3dp_Ni,
		m3dp_Nt,

		m3dp_map_Kd = 128, /* textured display map properties */
		m3dp_map_Ka,
		m3dp_map_Ks,
		m3dp_map_Ns,
		m3dp_map_Ke,
		m3dp_map_Tf,
		m3dp_map_Km, /* bump map */
		m3dp_map_D,
		m3dp_map_N, /* normal map */

		m3dp_map_Pr = 192, /* textured physical map properties */
		m3dp_map_Pm,
		m3dp_map_Ps,
		m3dp_map_Ni,
		m3dp_map_Nt,

		m3dp_bump = m3dp_map_Km,
		m3dp_map_il = m3dp_map_N,
		m3dp_refl = m3dp_map_Pm
	};

	 /* material property */
	[CRepr]
	public struct m3dp_t
	{
		/* property type, see "m3dp_*" enumeration */
		c_uchar type; [Union] struct
		{
			c_uint color; /* if value is a color, m3dpf_color */
			c_uint num; /* if value is a number, m3dpf_uint8, m3pf_uint16, m3dpf_uint32 */
			float    fnum; /* if value is a floating point number, m3dpf_float */
			M3D_INDEX textureid; /* if value is a texture, m3dpf_map */
		} value;
	}

	  /* material entry */
	[CRepr]
	public struct m3dm_t
	{
		c_char* name; /* name of the material */
		c_uchar numprop; /* number of properties */
		m3dp_t* prop; /* properties array */
	}

	 /* face entry */
	[CRepr]
	public struct m3df_t
	{
		M3D_INDEX materialid; /* material index */
		M3D_INDEX[3] vertex; /* 3D points of the triangle in CCW order */
		M3D_INDEX[3] normal; /* normal vectors */
		M3D_INDEX[3] texcoord; /* UV coordinates */
	 #if M3D_VERTEXMAX
		M3D_INDEX paramid;          /* parameter index */
		M3D_INDEX vertmax[3];       /* maximum 3D points of the triangle in CCW order */
	 #endif
	}

	[CRepr]
	public struct m3dvi_t
	{
		c_ushort count;
		c_char* name;
	}

	 /* voxel types (voxel palette) */
	[CRepr]
	public struct m3dvt_t
	{
		c_char* name; /* technical name of the voxel */
		c_uchar rotation; /* rotation info */
		c_ushort voxshape; /* voxel shape */
		M3D_INDEX materialid; /* material index */
		c_uint color; /* default voxel color */
		M3D_INDEX skinid; /* skin index */
		c_uchar numitem; /* number of sub-voxels */
		m3dvi_t* item; /* list of sub-voxels */
	}

	 /* voxel data blocks */
	[CRepr]
	public struct m3dvx_t
	{
		c_char* name; /* name of the block */
		c_int x, y, z; /* position */
		c_uint w, h, d; /* dimension */
		c_uchar uncertain; /* probability */
		c_uchar groupid; /* block group id */
		M3D_VOXEL* data; /* voxel data, indices to voxel type */
	}

	 /* shape command types. must match the row in m3d_commandtypes */
	enum shape_command_type
	{
		/* special commands */
		m3dc_use = 0, /* use material */
		m3dc_inc, /* include another shape */
		m3dc_mesh, /* include part of polygon mesh */
		/* approximations */
		m3dc_div, /* subdivision by constant resolution for both u, v */
		m3dc_sub, /* subdivision by constant, different for u and v */
		m3dc_len, /* spacial subdivision by maxlength */
		m3dc_dist, /* subdivision by maxdistance and maxangle */
		/* modifiers */
		m3dc_degu, /* degree for both u, v */
		m3dc_deg, /* separate degree for u and v */
		m3dc_rangeu, /* range for u */
		m3dc_range, /* range for u and v */
		m3dc_paru, /* u parameters (knots) */
		m3dc_parv, /* v parameters */
		m3dc_trim, /* outer trimming curve */
		m3dc_hole, /* inner trimming curve */
		m3dc_scrv, /* spacial curve */
		m3dc_sp, /* special points */
		/* helper curves */
		m3dc_bez1, /* Bezier 1D */
		m3dc_bsp1, /* B-spline 1D */
		m3dc_bez2, /* bezier 2D */
		m3dc_bsp2, /* B-spline 2D */
		/* surfaces */
		m3dc_bezun, /* Bezier 3D with control, UV, normal */
		m3dc_bezu, /* with control and UV */
		m3dc_bezn, /* with control and normal */
		m3dc_bez, /* control points only */
		m3dc_nurbsun, /* B-spline 3D */
		m3dc_nurbsu,
		m3dc_nurbsn,
		m3dc_nurbs,
		m3dc_conn, /* connect surfaces */
		/* geometrical */
		m3dc_line,
		m3dc_polygon,
		m3dc_circle,
		m3dc_cylinder,
		m3dc_shpere,
		m3dc_torus,
		m3dc_cone,
		m3dc_cube
	};

	 /* shape command argument types */
	enum shape_command_argument_type
	{
		m3dcp_mi_t = 1, /* material index */
		m3dcp_hi_t, /* shape index */
		m3dcp_fi_t, /* face index */
		m3dcp_ti_t, /* texture map index */
		m3dcp_vi_t, /* vertex index */
		m3dcp_qi_t, /* vertex index for quaternions */
		m3dcp_vc_t, /* coordinate or radius, float scalar */
		m3dcp_i1_t, /* int8 scalar */
		m3dcp_i2_t, /* int16 scalar */
		m3dcp_i4_t, /* int32 scalar */
		m3dcp_va_t /* variadic arguments */
	};

	const int M3D_CMDMAXARG = 8; /* if you increase this, add more arguments to the macro below */

	[CRepr]
	public struct m3dcd_t
	{

	 #if M3D_ASCII
		// #define M3D_CMDDEF(t,n,p,a,b,c,d,e,f,g,h) { (char*)(n), (p), { (a), (b), (c), (d), (e), (f), (g), (h) } }
		c_char* key;
	 #endif

	 #if !M3D_ASCII
		// #define M3D_CMDDEF(t,n,p,a,b,c,d,e,f,g,h) { (p), { (a), (b), (c), (d), (e), (f), (g), (h) } }
	 #endif
		c_uchar p;
		c_uchar[M3D_CMDMAXARG] a;
	};

	 /* shape command */
	[CRepr]
	public struct m3dc_t
	{
		c_ushort type; /* shape type */
		c_uint* arg; /* arguments array */
	};


	 /* shape entry */
	[CRepr]
	public struct  m3dh_t
	{
		c_char* name; /* name of the mathematical shape */
		M3D_INDEX group; /* group this shape belongs to or -1 */
		c_uint numcmd; /* number of commands */
		m3dc_t* cmd; /* commands array */
	}

	 /* label entry */
	[CRepr]
	public struct m3dl_t
	{
		c_char* name; /* name of the annotation layer or NULL */
		c_char* lang; /* language code or NULL */
		c_char* text; /* the label text */
		c_uint color; /* color */
		M3D_INDEX vertexid; /* the vertex the label refers to */
	}

	 /* frame transformations / working copy skeleton entry */
	[CRepr]
	public struct m3dtr_t
	{
		M3D_INDEX boneid; /* selects a node in bone hierarchy */
		M3D_INDEX pos; /* vertex index new position */
		M3D_INDEX ori; /* vertex index new orientation (quaternion) */
	};

	 /* animation frame entry */
	[CRepr]
	public struct m3dfr_t
	{
		c_uint msec; /* frame's position on the timeline, timestamp */
		M3D_INDEX numtransform; /* number of transformations in this frame */
		m3dtr_t* transform; /* transformations */
	};

	 /* model action entry */
	[CRepr] public struct m3da_t
	{
		c_char* name; /* name of the action */
		c_uint durationmsec; /* duration in millisec (1/1000 sec) */
		M3D_INDEX numframe; /* number of frames in this animation */
		m3dfr_t* frame; /* frames array */
	}

	 /* inlined asset */
	[CRepr]
	public struct m3di_t
	{
		c_char* name; /* asset name (same pointer as in texture[].name) */
		c_uchar* data; /* compressed asset data */
		c_uint length; /* compressed data length */
	}

	 /*** in-memory model structure ***/
	const int M3D_FLG_FREERAW    = (1 << 0);
	const int M3D_FLG_FREESTR    = (1 << 1);
	const int M3D_FLG_MTLLIB     = (1 << 2);
	const int M3D_FLG_GENNORM    = (1 << 3);

	[CRepr]
	public struct m3d_t
	{
		public m3dhdr_t* raw; /* pointer to raw data */
		public c_char flags; /* internal flags */
		public c_char errcode; /* returned error code */
		public c_char vc_s, vi_s, si_s, ci_s, ti_s, bi_s, nb_s, sk_s, fc_s, hi_s, fi_s, vd_s, vp_s; /* decoded sizes for types */
		public c_char* name; /* name of the model, like "Utah teapot" */
		public c_char* license; /* usage condition or license, like "MIT", "LGPL" or "BSD-3clause" */
		public c_char* author; /* nickname, email, homepage or github URL etc. */
		public c_char* desc; /* comments, descriptions. May contain '\n' newline character */
		public M3D_FLOAT scale; /* the model's bounding cube's size in SI meters */
		public M3D_INDEX numcmap;
		public c_uint* cmap; /* color map */
		public M3D_INDEX numtmap;
		public m3dti_t* tmap; /* texture map indices */
		public M3D_INDEX numtexture;
		public m3dtx_t* texture; /* uncompressed textures */
		public M3D_INDEX numbone;
		public m3db_t* bone; /* bone hierarchy */
		public M3D_INDEX numvertex;
		public m3dv_t* vertex; /* vertex data */
		public M3D_INDEX numskin;
		public m3ds_t* skin; /* skin data */
		public M3D_INDEX nummaterial;
		public m3dm_t* material; /* material list */
	 #if M3D_VERTEXMAX
		M3D_INDEX numparam;
		m3dvi_t *param;             /* parameters and their values list */
	 #endif
		public M3D_INDEX numface;
		public m3df_t* face; /* model face, polygon (triangle) mesh */
		public M3D_INDEX numvoxtype;
		public m3dvt_t* voxtype; /* model face, voxel types */
		public M3D_INDEX numvoxel;
		public m3dvx_t* voxel; /* model face, cubes compressed into voxels */
		public M3D_INDEX numshape;
		public m3dh_t* shape; /* model face, shape commands */
		public M3D_INDEX numlabel;
		public m3dl_t* label; /* annotation labels */
		public M3D_INDEX numaction;
		public m3da_t* action; /* action animations */
		public M3D_INDEX numinlined;
		public m3di_t* inlined; /* inlined assets */
		public M3D_INDEX numextra;
		public m3dchunk_t** extra; /* unknown chunks, application / engine specific data probably */
		public m3di_t preview; /* preview chunk */
	}

	 /*** export parameters ***/
	const int M3D_EXP_INT8      =  0;
	const int M3D_EXP_INT16     =  1;
	const int M3D_EXP_FLOAT     =  2;
	const int M3D_EXP_DOUBLE    =  3;

	const int M3D_EXP_NOCMAP     = (1 << 0);
	const int M3D_EXP_NOMATERIAL = (1 << 1);
	const int M3D_EXP_NOFACE     = (1 << 2);
	const int M3D_EXP_NONORMAL   = (1 << 3);
	const int M3D_EXP_NOTXTCRD   = (1 << 4);
	const int M3D_EXP_FLIPTXTCRD = (1 << 5);
	const int M3D_EXP_NORECALC   = (1 << 6);
	const int M3D_EXP_IDOSUCK    = (1 << 7);
	const int M3D_EXP_NOBONE     = (1 << 8);
	const int M3D_EXP_NOACTION   = (1 << 9);
	const int M3D_EXP_INLINE     = (1 << 10);
	const int M3D_EXP_EXTRA      = (1 << 11);
	const int M3D_EXP_NOZLIB     = (1 << 14);
	const int M3D_EXP_ASCII      = (1 << 15);
	const int M3D_EXP_NOVRTMAX   = (1 << 16);

	 /*** error codes ***/
	const int M3D_SUCCESS         =  0;
	const int M3D_ERR_ALLOC       = -1;
	const int M3D_ERR_BADFILE     = -2;
	const int M3D_ERR_UNIMPL      = -65;
	const int M3D_ERR_UNKPROP     = -66;
	const int M3D_ERR_UNKMESH     = -67;
	const int M3D_ERR_UNKIMG      = -68;
	const int M3D_ERR_UNKFRAME    = -69;
	const int M3D_ERR_UNKCMD      = -70;
	const int M3D_ERR_UNKVOX      = -71;
	const int M3D_ERR_TRUNC       = -72;
	const int M3D_ERR_CMAP        = -73;
	const int M3D_ERR_TMAP        = -74;
	const int M3D_ERR_VRTS        = -75;
	const int M3D_ERR_BONE        = -76;
	const int M3D_ERR_MTRL        = -77;
	const int M3D_ERR_SHPE        = -78;
	const int M3D_ERR_VOXT        = -79;

	/* callbacks */
	function c_uchar* m3dread_t(c_char* filename, c_uint* size); /* read file contents into buffer */
	function void m3dfree_t(void* buffer); /* free file contents buffer */
	function int m3dtxsc_t(c_char* name, void* script, c_uint len, m3dtx_t* output); /* interpret texture script */
	function int m3dprsc_t(c_char* name, void* script, c_uint len, m3d_t* model); /* interpret surface script */

	[CLink] public static extern m3d_t* m3d_load(c_uchar* data, m3dread_t readfilecb, m3dfree_t freecb, m3d_t* mtllib);

	[CLink] public static extern c_uchar* m3d_save(m3d_t* model, int quality, int flags, c_uint* size);

	[CLink] public static extern void m3d_free(m3d_t* model);

	[CLink] public static extern m3dtr_t* m3d_frame(m3d_t* model, M3D_INDEX actionid, M3D_INDEX frameid, m3dtr_t* skeleton);

	/* generate animation pose skeleton */
	[CLink] public static extern m3db_t* m3d_pose(m3d_t* model, M3D_INDEX actionid, c_uint msec);

	 /* private prototypes used by both importer and exporter */
	[CLink] public static extern c_char* _m3d_safestr(c_char* _in, int morelines);
}