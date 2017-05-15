/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/

#include "highlyreliablemarkers.h"
#include <iostream>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>

typedef std::vector<bool> Word;

class WordsLikeHood : public std::vector<double>
{
  std::vector<Word> words;
  std::vector<unsigned int> chosen;
  double totalPosibilities; //(2^n)^n - (2^n - 1)^n
  
public:
  WordsLikeHood(unsigned int wordSize);
  Word sampleWord();
  void incrementWord(Word w);
  void decrementWord(Word w);
  void update();  
  
  static unsigned int wordToNumber(Word w);
  static unsigned int wordTransitions(Word w);
  static Word wordFromNumber(unsigned int n, unsigned int bits);
  
};


using namespace std;

int main(int argc,char **argv)
{
   if(argc < 4) {
     cerr<<"Invalid number of arguments"<<endl;     
     cerr << "Usage: outputfile.yml dictSize n \n \
      outputfile.yml: output file for the dictionary \n \
      dictSize: number of markers to add to the dictionary \n \
      n: marker size." << endl;
     exit(-1);
   }  
  
    aruco::Dictionary D;
    unsigned int dictSize = atoi(argv[2]);
    unsigned int n = atoi(argv[3]);
    
    unsigned int tau = 2*( ( 4*( (n*n)/4 ) )/3 );
    cout << "Tau: " << tau << endl;
    
    WordsLikeHood P(n);
    
    vector<Word> wordsInM;
    wordsInM.resize(n);
    
    unsigned int countUnproductive=0;
    while(D.size() < dictSize) {
      aruco::MarkerCode candidate( n );
      for(unsigned int i=0; i<n; i++) {
	P.update();
	wordsInM[i] = P.sampleWord();
    for(unsigned int j=0; j<n; j++) candidate.set(i*n+j, wordsInM[i][j]);
	P.incrementWord(wordsInM[i]);
      }
//        for(unsigned int i=0; i<candidate.size(); i++) cout << candidate.get(i);
//        cout << endl;
      if( D.distance(candidate)>=tau && candidate.selfDistance()>=tau) {
	D.push_back(candidate);	
	cout << "Accepted Marker " << D.size() << "/" << dictSize << endl;
	cv::imshow("Marker", candidate.getImg(200) );
	countUnproductive=0;
      }
      else { // decrease words previously increased because they are not accepted!
    for(unsigned int i=0; i<n; i++) P.decrementWord(wordsInM[i]);
	countUnproductive++;
      }
      if(countUnproductive==5000) {
	tau--;
	countUnproductive=0;
	cout << "Reducing Tau to: " << tau << endl;
	if(tau==0) {
	  std::cerr << "Error: Tau=0. Small marker size for too high number of markers. Stop" << std::endl;
	  break;
	}	
      }
      char key = cv::waitKey(10);
      if(key==27) break;	
      if(key=='r') {
	tau--;
	cout << "Reducing Tau to: " << tau << endl;
      }
    }
    
  D.toFile(argv[1]);
  


}




WordsLikeHood::WordsLikeHood(unsigned int wordSize)
{
  resize( pow(float(2), float(wordSize)) );
  words.resize( size() );
  chosen.resize( size() );
  for(unsigned int i=0; i<size(); i++) {
    words[i] = WordsLikeHood::wordFromNumber(i, wordSize);
    chosen[i] = 0;
  }
  srand(time(NULL));
//   totalPosibilities = pow(2, wordSize*wordSize) - pow( pow(2,wordSize)-1.0 , wordSize); //(2^n)^n - (2^n - 1)^n
  totalPosibilities = 1;
  update();
}

void WordsLikeHood::incrementWord(Word w)
{
  assert( words.size()>0 && w.size()==words[0].size() );
  unsigned int id = WordsLikeHood::wordToNumber(w);
  chosen[id]++; 
  totalPosibilities++;
}

void WordsLikeHood::decrementWord(Word w)
{
  assert( words.size()>0 && w.size()==words[0].size() );
  unsigned int id = WordsLikeHood::wordToNumber(w);
  if(chosen[id]!=0) chosen[id]--; 
  totalPosibilities--;
}

void WordsLikeHood::update()
{
  for(unsigned int i=0; i<size(); i++) {
    (*this)[i] = ( 1.- ((double)chosen[i])/(double)totalPosibilities  ) * ((double)WordsLikeHood::wordTransitions(words[i])+1)/(double)(words[0].size());
  }
  
  double total = 0.0;
  for(unsigned int i=0; i<size(); i++) total+=(*this)[i];
  
  for(unsigned int i=0; i<size(); i++) {
    (*this)[i] /= total;
  }
    
}

Word WordsLikeHood::sampleWord()
{
  int randInt = rand();
  double randDouble = (double)randInt/(double)RAND_MAX;
  double acc = 0;
  for(unsigned int i=0; i<size(); i++) {
    acc += (*this)[i];
    if(acc >= randDouble) return words[i];
  }
  return Word();
}


unsigned int WordsLikeHood::wordToNumber(Word w)
{
  unsigned int id=0;
  unsigned int uintSize = sizeof(unsigned int)*8;
  for(unsigned int i=0; i<w.size() && i<uintSize; i++) {
    id += pow(float(2),float(i))*w[i];
  }    
  return id;  
}


unsigned int WordsLikeHood::wordTransitions(Word w)
{
    unsigned int count = 0;
    for(unsigned int i=0; i<w.size()-1; i++) {
      if(w[i]!=w[i+1]) count ++;
    }
    return count;  
}

Word WordsLikeHood::wordFromNumber(unsigned int n, unsigned int bits)
{
    if(pow(float(2),float(bits)) < n ) {
      std::cerr << "Not enough number of bits to codify id " << n << std::endl;
    }
    
    Word res;
    res.resize(bits);
    for(unsigned int i=0; i<bits; i++) {
      if(n & (unsigned int)pow(float(2),float(i))) res[i] = 1;
      else res[i] = 0;
    }  
    
    return res;
}

