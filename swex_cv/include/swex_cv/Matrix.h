#ifndef MATRIX_H
#define MATRIX_H

/**
 * \brief Une classe de matrice pleine
 */

template <class T>
class Matrix {
    protected:
        std::vector<std::vector<T> > data;
    public:

        /**
         * \brief Constructeur par defaut
         */
        Matrix(){}

        /**
         * \brief Alloue en memoire une matrice
         * \param nb_lig Nombre de ligne
         * \param nb_col Nombre de colonne
         */
        Matrix(std::size_t nb_lig,std::size_t nb_col){
            for(std::size_t lig=0;lig<nb_lig;++lig){
                std::vector<T> tmp;
                tmp.reserve(nb_col);
                for(std::size_t col=0;col<nb_col;++col){
                    tmp.push_back(0);//valeur par defaut
                }
                data.push_back(tmp);
            }
        }

        /**
         * \brief Destructeur
         */
        ~Matrix(){}

        /**
         * \return Le nombre de ligne
         */
        inline std::size_t rows() const{
            return data.size();
        }

        /**
         * \return Le nombre de colonne
         */
        inline std::size_t cols() const{
            if (data.size()==0) return 0;
            return data[0].size();
        }

        /**
         * \brief Accesseur vers un element de la matrice
         * \param lig son numero de ligne
         * \param col son numero de colonne
         * \return la valeur stockee
         */
        inline T get(std::size_t lig,std::size_t col) const{
            assert(lig<rows());
            assert(col<cols());
            return data[lig][col];
        }

        /**
         * \brief Accesseur vers un element de la matrice
         * \param lig son numero de ligne
         * \param col son numero de colonne
         * \param val la valeur a stocker
         */
        inline void set(std::size_t lig,std::size_t col,T val){
            assert(lig<rows());
            assert(col<cols());
            data[lig][col]=val;
        }

};

/**
 * \brief Operateur << pour les matrices
 * \param o Le flux de sortie
 * \param m la matrice a ecrire
 */
template <class T>
std::ostream& operator << (std::ostream& o,const Matrix<T> & m){
    for(std::size_t lig=0;lig<m.rows();++lig){
        for(std::size_t col=0;col<m.cols();++col){
            o<<m.get(lig,col)<<" ";
        }
        o<<std::endl;
    }
    return o;
}

/**
 * \brief Addition matricielle
 * \param m1 la premiere matrice
 * \param m2 la deuxieme matrice
 * \return la somme
 */
template <class T>
Matrix<T> operator+ (const Matrix<T> & m1,const Matrix<T> & m2){
    assert(m1.rows()==m1.rows());
    assert(m1.cols()==m1.cols());
    std::size_t nb_lig=m1.rows();
    std::size_t nb_col=m1.cols();
    Matrix<T> sum(nb_lig,nb_col);
    for(std::size_t lig=0;lig<nb_lig;++lig){
        for(std::size_t col=0;col<nb_col;++col){
            sum.set(lig,col,m1.get(lig,col)+m2.get(lig,col));
        }
    }
    return sum;
}

/**
 * \brief Multiplication matricielle
 * \param m1 la premiere matrice
 * \param m2 la deuxieme matrice
 * \return le produit
 */
template <class T>
Matrix<T> operator* (const Matrix<T> & m1,const Matrix<T> & m2){

    assert(m1.cols()==m2.rows());
    std::size_t nb_lig=m1.rows();
    std::size_t nb_col=m2.cols();
    Matrix<T> prod(nb_lig,nb_col);
    for(std::size_t lig=0;lig<nb_lig;++lig){
        for(std::size_t col=0;col<nb_col;++col){
            //calcul du terme (lig,col)
            T tmp=0;
            for(std::size_t i=0;i<m1.cols();++i){
                tmp+=m1.get(lig,i)*m2.get(i,col);
            }
            prod.set(lig,col,tmp);
        }
    }
    return prod;
}

#endif // MATRIX_H
