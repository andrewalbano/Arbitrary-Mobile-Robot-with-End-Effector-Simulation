//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//

function matrix_multiply(m1,m2) {
    // returns 2D array that is the result of m1*m2
    // get dimensions of each matrix
    let num_rows_1 = get_num_rows(m1)
    let num_columns_1 = get_num_columns(m1)
    let num_rows_2 = get_num_rows(m2)
    let num_columns_2 = get_num_columns(m2)

    //console.log("Resulting dimensions of multiplication are : " + num_rows_1 , ", ",  + num_columns_2)
    m = new Array(num_rows_1) ///initializing an array with this many rows
    // make sure dimensions agree
    if (num_columns_1 != num_rows_2){
        return "matrix indices are not compatible"
    }
    else{
       
        for (let i = 0; i <num_rows_1; i++){ 
            m[i] = new Array(num_columns_2) 
            for (let j = 0; j <num_columns_2; j++){ 
                m[i][j] = 0; 
                for(let k = 0; k <num_columns_1; k++)
                m[i][j] += m1[i][k] * m2[k][j] 
            }
            //https://stackoverflow.com/questions/27205018/multiply-2-matrices-in-javascript
        }
        
    }
    return m
}
// student implemented helper functions
function get_value(row,col,m) {
    // returns value at the index
    return m[row][col]
}
function get_num_rows(m) {
    // returns num of rows
    return m.length
}
function get_num_columns(m) {
    // returns num of rows
    return m[0].length
}

function matrix_transpose(m) {
    // returns 2D array that is the result of m1 tranpose, but does not alter the original matrix
    /*
    m_T = m;
    
    let num_rows = get_num_rows(m)
    let num_columns = get_num_columns(m)

    for (let i = 0; i <num_rows; i++){
        for(let j = 0; j<i; j++){
            // swap element [i,j] and [j,i]
            let temp = m_T[i][j];
            m_T[i][j] = m_T[j][i];
            m_T[j][i] = temp;
        }
    }*/
        let m_T = m.reduce((prev, next) => 
                next.map((item, i) =>
        (prev[i] || []).concat(next[i])
    ), []);

    return m_T
}

function matrix_pseudoinverse(m) {
    // returns pseudoinverse of matrix m
        
        // get dimensions
        let N = get_num_rows(m)
        let M = get_num_columns(m)

        //compute transpose
        let mT = matrix_transpose(m)
        
        //determine which pseudo to take
        if(N > M){ // left pseudo
            let pseudo = matrix_multiply(numeric.inv(matrix_multiply(mT,m)),mT)
            return pseudo
        }
        else if(N < M){ // right pseudo
            let pseudo = matrix_multiply(mT , numeric.inv(matrix_multiply(m,mT)))
            return pseudo
        }     
}

function matrix_invert_affine(m) {
    // returns 2D array that is the invert affine of 4-by-4 matrix m
    let R = []
    let temp0 = []
    let temp1 = []
    let temp2 = []

    for(j=0; j<3; j+=1){
        temp0.push(m[0][j])
        temp1.push(m[1][j])
        temp2.push(m[2][j])
    }

    R.push(temp0)
    R.push(temp1)
    R.push(temp2)

    let R_inverse = matrix_transpose(R)
    R_inverse[0].push(0)
    R_inverse[1].push(0)
    R_inverse[2].push(0)

    R_inverse.push([0,0,0,1])

    let T = [ [m[0][3]], [m[1][3]], [m[2][3]], [1]]

    let T_prime = matrix_multiply(R_inverse,T)

    let m_inverse = R_inverse
    m_inverse[0][3] = -1*T_prime[0][0]
    m_inverse[1][3] = -1*T_prime[1][0]
    m_inverse[2][3] = -1*T_prime[2][0]


    return m_inverse

}
function vector_normalize(v) {
    // returns normalized vector for v
    let sum = 0
    for (let i = 0; i < v.length; i++){
        sum += (v[i])**2
    }   

    magnitude = Math.sqrt(sum)
    v_normalized = []

    for (let i = 0; i < v.length; i++){
        v_normalized.push((1/magnitude)*v[i])
    }   

    return v_normalized
}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions
    let result = []
    result.push(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
    return result
}

function generate_ones(m,n){
    let matrix = []
    let temp =[]
    for(j=0; j<n; j+=1){
        temp.push(1)
    }

    for (i=0; i<m; i+=1){
        
        matrix.push(temp) 
    }
    return matrix   

}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    let I = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    return I
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    let T = generate_identity()
    T[0][3] = tx
    T[1][3] = ty
    T[2][3] = tz
    return T
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    R = generate_identity()
    R[1][1] = Math.cos(angle)
    R[1][2] = -Math.sin(angle)
    R[2][1] = Math.sin(angle)
    R[2][2] = Math.cos(angle)
    
    return R
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    R = generate_identity()
    R[0][0] = Math.cos(angle)
    R[0][2] = Math.sin(angle)
    R[2][0] = -Math.sin(angle)
    R[2][2] = Math.cos(angle)
    
    return R
}

function generate_rotation_matrix_Z(angle) {
    //     // returns 4-by-4 matrix as a 2D array, angle is in radians
    R = generate_identity()
    R[0][0] = Math.cos(angle)
    R[0][1] = -Math.sin(angle)
    R[1][0] = Math.sin(angle)
    R[1][1] = Math.cos(angle)
    
    return R
}

function generate2D(rows,columns){
    let my2DArray = Array.from(Array(rows), () => new Array(columns).fill(0));
    
    return my2DArray
}
