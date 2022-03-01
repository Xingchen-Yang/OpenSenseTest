#This code is copied from https://github.com/jimmyDunne/opensimIMUTracking, transfer to python
import opensim as osim
def rotationOrientationTable(oTable, R):
    #Get table properties
    nc = oTable.getNumColumns();
    nt = oTable.getNumRows();
    #Get the Data Times
    times = oTable.getIndependentColumn();
    #Make a empty Matrix of Rotations
    matrix = osim.MatrixRotation(nt, nc, osim.Rotation());
    # Perform the Rotations
    for i in range(0, nt):
        row = oTable.getRowAtIndex(i);
        for j in range(0, nc):
            matrix.set(i,j, R.multiply(row.getElt(0,j))); #not right
    # Convert Matrix into a TimesSeriesTable     
    rotOTable = osim.TimeSeriesTableRotation(times, matrix, oTable.getColumnLabels());
    # Copy the Metadata
    for metaKey in oTable.getTableMetaDataKeys():
        rotOTable.addTableMetaDataString(metaKey, oTable.getTableMetaDataString(metaKey));
        
    return rotOTable