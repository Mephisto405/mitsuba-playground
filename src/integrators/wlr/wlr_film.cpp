#include <mitsuba/render/film.h>

MTS_NAMESPACE_BEGIN

/// A film defines how conducted measurements are stored and 
/// converted into the final output file that is written to 
/// disk at the end of the rendering process.

/// WLR_FilmÀº 
class WLR_Film : public Film {
public:
	/// Create a film
	WLR_Film(const Properties &props) : Film(props) {

		int nPix = m_cropSize.x * m_cropSize.y;

		// Basic Buffers - initialization
		m_mapSPP = (int*)calloc(nPix, sizeof(int));
		m_accImg = (float*)calloc(nPix * 3, sizeof(float));
		m_accImg2 = (float*)calloc(nPix * 3, sizeof(float));
		m_accNormal = (float*)calloc(nPix * 3, sizeof(float));
		m_accNormal2 = (float*)calloc(nPix * 3, sizeof(float));
		m_accTexture = (float*)calloc(nPix * 3, sizeof(float));
		m_accTexture2 = (float*)calloc(nPix * 3, sizeof(float));
		m_accDepth = (float*)calloc(nPix, sizeof(float));
		m_accDepth2 = (float*)calloc(nPix, sizeof(float));
	}
	/// Unserialize a film
	WLR_Film(Stream *stream, InstanceManager *manager) : Film(stream, manager) {
	
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Film::serialize(stream, manager);
		stream->write
	}
	/// Destructor
	~WLR_Film() {
		free(m_accImg);
		free(m_accImg2);
		free(m_accNormal);
		free(m_accNormal2);
		free(m_accTexture);
		free(m_accTexture2);
		free(m_accDepth);
		free(m_accDepth2);
		free(m_mapSPP);
	}

	void addBitmap(const Bitmap *bitmap, Float multiplier) {
		Film::addBitmap(bitmap, multiplier);
	}


	MTS_DECLARE_CLASS()
private:
	// Input Buffers 
	float* m_accImg;
	float* m_accImg2;
	float* m_accNormal;
	float* m_accNormal2;
	float* m_accTexture;
	float* m_accTexture2;
	float* m_accDepth;
	float* m_accDepth2;
	int* m_mapSPP;
};

MTS_IMPLEMENT_CLASS_S(WLR_Film, false, Film)
MTS_EXPORT_PLUGIN(WLR_Film, "Film for Weighted Local Regression");
MTS_NAMESPACE_END