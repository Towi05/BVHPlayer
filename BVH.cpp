/**
***  BVH動作ファイ?の読み?み・描画ク?ス
***  Copyright (c) 2004-2007, Masaki OSHITA (www.oshita-lab.org)
**/


#include <fstream>
#include <string.h>

#include "BVH.h"


// コ?ト?クタ
BVH::BVH()
{
	motion = NULL;
	Clear();
}

// コ?ト?クタ
BVH::BVH( const char * bvh_file_name )
{
	motion = NULL;
	Clear();

	Load( bvh_file_name );
}

// デスト?クタ
BVH::~BVH()
{
	Clear();
}


// 全情報のク?ア
void  BVH::Clear()
{
	int  i;
	for ( i=0; i<channels.size(); i++ )
		delete  channels[ i ];
	for ( i=0; i<joints.size(); i++ )
		delete  joints[ i ];
	if ( motion != NULL )
		delete  motion;

	is_load_success = false;
	
	file_name = "";
	motion_name = "";
	
	num_channel = 0;
	channels.clear();
	joints.clear();
	joint_index.clear();
	
	num_frame = 0;
	interval = 0.0;
	motion = NULL;
}



//
//  BVHファイ?の?ード
//
void  BVH::Load( const char * bvh_file_name )
{
	#define  BUFFER_LENGTH  1024*4

	ifstream  file;
	char      line[ BUFFER_LENGTH ];
	char *    token;
	char      separater[] = " :,\t";
	vector< Joint * >   joint_stack;
	Joint *   joint = NULL;
	Joint *   new_joint = NULL;
	bool      is_site = false;
	double    x, y ,z;
	int       i, j;

	// ?期化
	Clear();

	// ファイ?の情報（ファイ?名・動作名）の設定
	file_name = bvh_file_name;
	const char *  mn_first = bvh_file_name;
	const char *  mn_last = bvh_file_name + strlen( bvh_file_name );
	if ( strrchr( bvh_file_name, '\\' ) != NULL )
		mn_first = strrchr( bvh_file_name, '\\' ) + 1;
	else if ( strrchr( bvh_file_name, '/' ) != NULL )
		mn_first = strrchr( bvh_file_name, '/' ) + 1;
	if ( strrchr( bvh_file_name, '.' ) != NULL )
		mn_last = strrchr( bvh_file_name, '.' );
	if ( mn_last < mn_first )
		mn_last = bvh_file_name + strlen( bvh_file_name );
	motion_name.assign( mn_first, mn_last );

	// ファイ?のオープ?
	file.open( bvh_file_name, ios::in );
	if ( file.is_open() == 0 )  return; // ファイ?が開けなかったら終了

	// 階層情報の読み?み
	while ( ! file.eof() )
	{
		// ファイ?の最後まできてしまったら異常終了
		if ( file.eof() )  goto bvh_error;

		// １行読み?み、先頭の単語を取得
		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, separater );

		// 空行の場?は?の行へ
		if ( token == NULL )  continue;

		// 関節ブ?ックの開始
		if ( strcmp( token, "{" ) == 0 )
		{
			// 現在の関節をスタックに積む
			joint_stack.push_back( joint );
			joint = new_joint;
			continue;
		}
		// 関節ブ?ックの終了
		if ( strcmp( token, "}" ) == 0 )
		{
			// 現在の関節をスタックから取り出す
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}

		// 関節情報の開始
		if ( ( strcmp( token, "ROOT" ) == 0 ) ||
		     ( strcmp( token, "JOINT" ) == 0 ) )
		{
			// 関節データの作成
			new_joint = new Joint();
			new_joint->index = joints.size();
			new_joint->parent = joint;
			new_joint->has_site = false;
			new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
			new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
			joints.push_back( new_joint );
			if ( joint )
				joint->children.push_back( new_joint );

			// 関節名の読み?み
			token = strtok( NULL, "" );
			while ( *token == ' ' )  token ++;
			new_joint->name = token;

			// イ?デックスへ追加
			joint_index[ new_joint->name ] = new_joint;
			continue;
		}

		// ?端情報の開始
		if ( ( strcmp( token, "End" ) == 0 ) )
		{
			new_joint = joint;
			is_site = true;
			continue;
		}

		// 関節のオフセット or ?端位置の情報
		if ( strcmp( token, "OFFSET" ) == 0 )
		{
			// 座標値を読み?み
			token = strtok( NULL, separater );
			x = token ? atof( token ) : 0.0;
			token = strtok( NULL, separater );
			y = token ? atof( token ) : 0.0;
			token = strtok( NULL, separater );
			z = token ? atof( token ) : 0.0;
			
			// 関節のオフセットに座標値を設定
			if ( is_site )
			{
				joint->has_site = true;
				joint->site[0] = x;
				joint->site[1] = y;
				joint->site[2] = z;
			}
			else
			// ?端位置に座標値を設定
			{
				joint->offset[0] = x;
				joint->offset[1] = y;
				joint->offset[2] = z;
			}
			continue;
		}

		// 関節のチ??ネ?情報
		if ( strcmp( token, "CHANNELS" ) == 0 )
		{
			// チ??ネ??を読み?み
			token = strtok( NULL, separater );
			joint->channels.resize( token ? atoi( token ) : 0 );

			// チ??ネ?情報を読み?み
			for ( i=0; i<joint->channels.size(); i++ )
			{
				// チ??ネ?の作成
				Channel *  channel = new Channel();
				channel->joint = joint;
				channel->index = channels.size();
				channels.push_back( channel );
				joint->channels[ i ] = channel;

				// チ??ネ?の種類の判定
				token = strtok( NULL, separater );
				if ( strcmp( token, "Xrotation" ) == 0 )
					channel->type = X_ROTATION;
				else if ( strcmp( token, "Yrotation" ) == 0 )
					channel->type = Y_ROTATION;
				else if ( strcmp( token, "Zrotation" ) == 0 )
					channel->type = Z_ROTATION;
				else if ( strcmp( token, "Xposition" ) == 0 )
					channel->type = X_POSITION;
				else if ( strcmp( token, "Yposition" ) == 0 )
					channel->type = Y_POSITION;
				else if ( strcmp( token, "Zposition" ) == 0 )
					channel->type = Z_POSITION;
			}
		}

		// Motionデータのセクシ??へ移る
		if ( strcmp( token, "MOTION" ) == 0 )
			break;
	}


	// ?ーシ??情報の読み?み
	file.getline( line, BUFFER_LENGTH );
	token = strtok( line, separater );
	if ( strcmp( token, "Frames" ) != 0 )  goto bvh_error;
	token = strtok( NULL, separater );
	if ( token == NULL )  goto bvh_error;
	num_frame = atoi( token );

	file.getline( line, BUFFER_LENGTH );
	token = strtok( line, ":" );
	if ( strcmp( token, "Frame Time" ) != 0 )  goto bvh_error;
	token = strtok( NULL, separater );
	if ( token == NULL )  goto bvh_error;
	interval = atof( token );

	num_channel = channels.size();
	motion = new double[ num_frame * num_channel ];

	// ?ーシ??データの読み?み
	for ( i=0; i<num_frame; i++ )
	{
		file.getline( line, BUFFER_LENGTH );
		token = strtok( line, separater );
		for ( j=0; j<num_channel; j++ )
		{
			if ( token == NULL )
				goto bvh_error;
			motion[ i*num_channel + j ] = atof( token );
			token = strtok( NULL, separater );
		}
	}

	// ファイ?のク?ーズ
	file.close();

	// ?ードの成功
	is_load_success = true;

	return;

bvh_error:
	file.close();
}



//
//  BVH?格・姿勢の描画関?
//

#include <math.h>
#include <gl/glut.h>


// 指定フ?ー?の姿勢を描画
void  BVH::RenderFigure( int frame_no, float scale )
{
	// BVH?格・姿勢を指定して描画
	RenderFigure( joints[ 0 ], motion + frame_no * num_channel, scale );
}


// 指定されたBVH?格・姿勢を描画（ク?ス関?）
void  BVH::RenderFigure( const Joint * joint, const double * data, float scale )
{
	glPushMatrix();

	// ?ート関節の場?は平行移動を適用
	if ( joint->parent == NULL )
	{
		glTranslatef( data[ 0 ] * scale, data[ 1 ] * scale, data[ 2 ] * scale );
	}
	// 子関節の場?は親関節からの平行移動を適用
	else
	{
		glTranslatef( joint->offset[ 0 ] * scale, joint->offset[ 1 ] * scale, joint->offset[ 2 ] * scale );
	}

	// 親関節からの回転を適用（?ート関節の場?は?ー?ド座標からの回転）
	int  i, j;
	for ( i=0; i<joint->channels.size(); i++ )
	{
		Channel *  channel = joint->channels[ i ];
		if ( channel->type == X_ROTATION )
			glRotatef( data[ channel->index ], 1.0f, 0.0f, 0.0f );
		else if ( channel->type == Y_ROTATION )
			glRotatef( data[ channel->index ], 0.0f, 1.0f, 0.0f );
		else if ( channel->type == Z_ROTATION )
			glRotatef( data[ channel->index ], 0.0f, 0.0f, 1.0f );
	}

	// ??クを描画
	// 関節座標系の原点から?端点への??クを描画
	if ( joint->children.size() == 0 )
	{
		RenderBone( 0.0f, 0.0f, 0.0f, joint->site[ 0 ] * scale, joint->site[ 1 ] * scale, joint->site[ 2 ] * scale );
	}
	// 関節座標系の原点から?の関節への接続位置への??クを描画
	if ( joint->children.size() == 1 )
	{
		Joint *  child = joint->children[ 0 ];
		RenderBone( 0.0f, 0.0f, 0.0f, child->offset[ 0 ] * scale, child->offset[ 1 ] * scale, child->offset[ 2 ] * scale );
	}
	// 全関節への接続位置への?心点から各関節への接続位置へ円?を描画
	if ( joint->children.size() > 1 )
	{
		// 原点と全関節への接続位置への?心点を計算
		float  center[ 3 ] = { 0.0f, 0.0f, 0.0f };
		for ( i=0; i<joint->children.size(); i++ )
		{
			Joint *  child = joint->children[ i ];
			center[ 0 ] += child->offset[ 0 ];
			center[ 1 ] += child->offset[ 1 ];
			center[ 2 ] += child->offset[ 2 ];
		}
		center[ 0 ] /= joint->children.size() + 1;
		center[ 1 ] /= joint->children.size() + 1;
		center[ 2 ] /= joint->children.size() + 1;

		// 原点から?心点への??クを描画
		RenderBone(	0.0f, 0.0f, 0.0f, center[ 0 ] * scale, center[ 1 ] * scale, center[ 2 ] * scale );

		// ?心点から?の関節への接続位置への??クを描画
		for ( i=0; i<joint->children.size(); i++ )
		{
			Joint *  child = joint->children[ i ];
			RenderBone(	center[ 0 ] * scale, center[ 1 ] * scale, center[ 2 ] * scale,
				child->offset[ 0 ] * scale, child->offset[ 1 ] * scale, child->offset[ 2 ] * scale );
		}
	}

	// 子関節に対して再帰呼び出し
	for ( i=0; i<joint->children.size(); i++ )
	{
		RenderFigure( joint->children[ i ], data, scale );
	}

	glPopMatrix();
}


// BVH?格の１本の??クを描画（ク?ス関?）
void  BVH::RenderBone( float x0, float y0, float z0, float x1, float y1, float z1 )
{
	// 与えられた２点を?ぶ円?を描画

	// 円?の２端点の情報を原点・向き・長さの情報に変換
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );

	// 描画パ??タの設定
	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();

	// 平行移動を設定
	glTranslated( x0, y0, z0 );

	// 以下、円?の回転を表す行列を計算

	// ?軸を単位ベクト?に正規化
	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	// 基?とする?軸の向きを設定
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	// ?軸と?軸の外積から?軸の向きを計算
	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	// ?軸を単位ベクト?に正規化
	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	// ?軸と?軸の外積から?軸の向きを計算
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	// 回転行列を設定
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
	                    up_x,   up_y,   up_z,   0.0,
	                    dir_x,  dir_y,  dir_z,  0.0,
	                    0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );

	// 円?の設定
	GLdouble radius= 0.01; // 円?の太さ
	GLdouble slices = 8.0; // 円?の放射状の細分?（デフォ?ト12）
	GLdouble stack = 3.0;  // 円?の輪切りの細分?（デフォ?ト１）

	// 円?を描画
	gluCylinder( quad_obj, radius, radius, bone_length, slices, stack ); 
	
	glPopMatrix();
}



// End of BVH.cpp
