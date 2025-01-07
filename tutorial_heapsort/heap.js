/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
  
    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation

    //add new element to end of tree
    heap.push(new_element); 
    let index = heap.length -1; //get the index of the new element


    // if heap condidion is not satisfied swap inserted node with parent node
    // heap condition: sort min to max
    while (index > 0){ //repeat until reaches the first position, unless condition is satisfied to break
        const parentIndex = Math.floor((index-1)/2); // get parent index. rounds up when necessary using floor
        if(heap[index]>=heap[parentIndex]){ // compare current index and parent. if it is greater than parent, break
            break;
        }
        else{ //rearranging heap by swapping the values returned by parent index and current index
            [heap[index],heap[parentIndex]] =[heap[parentIndex], heap[index]];
            index = parentIndex; 
        }
    }
    
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation

    //if the last element is reached, return it
    if(heap.length == 1){
        extractedValue = heap.pop();
        return extractedValue;
    }

    // extract root element, then replace it with the last element
    extractedValue = heap[0];
    heap[0] = heap.pop();

    //swap with higher priority child (heapify)
    let index = 0; //index of new root element
    
    while(true){
               
        leftIndex = 2*index+1;
        rightIndex = 2*index+2;
        length = heap.length-1;
        childIndex = null; // smallest child index
        
        // determine priority child

        //if left child exists make it the smallest child index
        if(leftIndex <= length){//} && heap[leftIndex] < heap[index]){
            childIndex = leftIndex;
        }
         //if right child exists, compare it to the left index and update the child index
         if(rightIndex <= length && heap[rightIndex] < heap[leftIndex]){
            childIndex = rightIndex;
        }
        // if the inital root is smaller than both children then break, otherwise heapify repeatedly
        if (childIndex == null || heap[index] <= heap[childIndex]){
            break;
        }
        else {//swap index and child index
            [heap[index],heap[childIndex]] =[heap[childIndex], heap[index]];
            index = childIndex; //reassign the index of the element that was added if the position changed.
        }
    }
    
    return extractedValue;   
}

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
    minheaper.extract = minheap_extract;






