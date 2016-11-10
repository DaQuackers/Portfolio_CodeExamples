using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.IO.Pipes;
using FullSailAFI.GamePlaying.CoreAI;

namespace FullSailAFI.GamePlaying
{
    public class StudentAI : Behavior
    {
        TreeVisLib treeVisLib;  // lib functions to communicate with TreeVisualization
        bool visualizationFlag = false;  // turn this on to use tree visualization (which you will have to implement via the TreeVisLib API)
        // WARNING: Will hang program if the TreeVisualization project is not loaded!

        public StudentAI()
        {
            if (visualizationFlag == true)
            {
                if (treeVisLib == null)  // should always be null, but just in case
                    treeVisLib = TreeVisLib.getTreeVisLib();  // WARNING: Creation of this object will hang if the TreeVisualization project is not loaded!
            }
        }

        //
        // This function starts the look ahead process to find the best move
        // for this player color.
        //
        public ComputerMove Run(int _nextColor, Board _board, int _lookAheadDepth)
        {
            ComputerMove nextMove = GetBestMove(_nextColor, _board, _lookAheadDepth);
            return nextMove;
        }

        ////////////////////////////////////////
        /*
                Min-Max Algorithm
        */
        ////////////////////////////////////////
        private ComputerMove GetBestMove(int color, Board board, int depth)
        {
            ComputerMove bestMove = null;
            Board newBoard = new Board();
            List<ComputerMove> moveslist = new List<ComputerMove>();
            for (int i = 0; i < Board.Height; ++i)
            {
                for (int j = 0; j < Board.Width; ++j)
                {
                    if (board.IsValidMove(color, i, j))
                    {
                        moveslist.Add(new ComputerMove(i, j));
                    }
                }
            }
            for (int i = 0; i < moveslist.Count; ++i)
            {
                newBoard.Copy(board);
                newBoard.MakeMove(color, moveslist[i].row, moveslist[i].col);
                if (newBoard.IsTerminalState() || depth == 0)
                    moveslist[i].rank = Evaluate(newBoard);
                else if (!newBoard.HasAnyValidMove(-color))
                    moveslist[i].rank = GetBestMove(color, newBoard, depth - 1).rank;
                else
                    moveslist[i].rank = GetBestMove(-color, newBoard, depth - 1).rank;
                if (bestMove == null || (color == Board.Black && moveslist[i].rank > bestMove.rank) || (color == Board.White && moveslist[i].rank < bestMove.rank))
                    bestMove = moveslist[i];
            }
            return bestMove;
        }

        #region Recommended Helper Functions
        private int Evaluate(Board _board)
        {
            int ret = 0;
            for (int i = 0; i < Board.Height; ++i)
                for (int j = 0; j < Board.Width; ++j)
                {
                    int val = -1 * _board.GetSquareContents(i, j);
                    if (i == Board.Height - 1 || i == 0)
                        val *= 10;
                    if (j == Board.Width - 1 || j == 0)
                        val *= 10;
                    ret += val;
                }
            bool term = _board.IsTerminalState();
            if (ret > 0 && term)
                ret += 10000;
            else if (ret < 0 && term)
                ret -= 10000;
            return ret;
        }

        #endregion

    }
}
